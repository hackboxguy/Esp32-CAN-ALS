#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <signal.h>

class CANCalibrator;
static CANCalibrator* g_calibrator = nullptr;

class CANCalibrator {
private:
    int can_socket;
    std::atomic<bool> response_received{false};
    std::atomic<bool> in_calibration_mode{false};
    struct can_frame last_response;
    struct can_frame last_raw_data;
    struct can_frame last_lux_data;

    // CAN IDs
    static constexpr canid_t ID_CALIBRATION_CMD = 0x0A3;
    static constexpr canid_t ID_CALIBRATION_RESP = 0x0A4;
    static constexpr canid_t ID_RAW_SENSOR_DATA = 0x0A5;
    static constexpr canid_t ID_LUX_DATA = 0x0A2;

    // Commands
    static constexpr uint8_t CAL_CMD_ENTER = 0x01;
    static constexpr uint8_t CAL_CMD_SET_REFERENCE = 0x02;
    static constexpr uint8_t CAL_CMD_SAVE = 0x03;
    static constexpr uint8_t CAL_CMD_EXIT = 0x04;
    static constexpr uint8_t CAL_CMD_GET_OFFSET = 0x05;
    static constexpr uint8_t CAL_CMD_RESET = 0x06;
    static constexpr uint8_t CAL_CMD_SET_REF_LOW = 0x07;
    static constexpr uint8_t CAL_CMD_SET_REF_MED = 0x08;
    static constexpr uint8_t CAL_CMD_SET_REF_HIGH = 0x09;
    static constexpr uint8_t CAL_CMD_SET_REF_VERYHIGH = 0x0A;
    static constexpr uint8_t CAL_CMD_GET_ALL_OFFSETS = 0x0B;

    const char* range_names[4] = {"LOW", "MED", "HIGH", "V_HIGH"};

    struct RangeInfo {
        uint16_t min_lux;
        uint16_t max_lux;
        uint16_t optimal_min;
        uint16_t optimal_max;
        const char* description;
    };

    /*const RangeInfo range_info[4] = {
        {1,   50,   5,   35,  "Very dim (night, parking lights)"},
        {10,  350,  20,  250, "Normal indoor lighting"},
        {100, 1200, 200, 800, "Bright indoor to daylight"},
        {500, 5000, 800, 2000, "Direct sunlight, very bright conditions"}
    };*/

const RangeInfo range_info[4] = {
    {1,   50,   5,   35,  "Very dim (night, parking lights)"},        // LOW: matches firmware (0-35)
    {10,  1200, 20,  250, "Normal indoor lighting"},                   // MEDIUM: increased max from 900 to 1200
    {200, 1350, 400, 800, "Bright indoor to daylight"},               // HIGH: adjusted min from 100 to 200
    {600, 5000, 800, 2000, "Direct sunlight, very bright conditions"} // V_HIGH: matches firmware (600+)
};
public:
    std::atomic<bool> running{true};
    std::atomic<bool> interrupt_requested{false};

    CANCalibrator() : can_socket(-1) {
        g_calibrator = this;
    }

    ~CANCalibrator() {
        cleanup();
    }

    void cleanup() {
        if (in_calibration_mode && can_socket >= 0) {
            std::cout << "\n[INFO] Exiting calibration mode..." << std::endl;
            send_command_simple(CAL_CMD_EXIT);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        running = false;
        if (can_socket >= 0) {
            close(can_socket);
            can_socket = -1;
        }
        in_calibration_mode = false;
    }

    bool initialize(const char* interface = "can0") {
        can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket < 0) {
            std::cerr << "[ERROR] Cannot create CAN socket" << std::endl;
            return false;
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, interface);
        ioctl(can_socket, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "[ERROR] Cannot bind to CAN interface " << interface << std::endl;
            close(can_socket);
            can_socket = -1;
            return false;
        }

        return true;
    }

    void listen_thread() {
        struct can_frame frame;

        while (running) {
            int nbytes = read(can_socket, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                continue;
            }

            if (frame.can_id == ID_CALIBRATION_RESP) {
                last_response = frame;
                response_received = true;
            } else if (frame.can_id == ID_RAW_SENSOR_DATA) {
                last_raw_data = frame;
            } else if (frame.can_id == ID_LUX_DATA) {
                last_lux_data = frame;
            }
        }
    }

    bool send_command(uint8_t command, const uint8_t* data = nullptr, size_t data_len = 0) {
        struct can_frame frame;
        frame.can_id = ID_CALIBRATION_CMD;
        frame.can_dlc = 8;
        memset(frame.data, 0, 8);

        frame.data[0] = command;
        if (data && data_len > 0) {
            memcpy(&frame.data[1], data, std::min(data_len, size_t(7)));
        }

        response_received = false;

        int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
        return (nbytes == sizeof(struct can_frame));
    }

    bool send_command_simple(uint8_t command) {
        if (can_socket < 0) return false;

        struct can_frame frame;
        frame.can_id = ID_CALIBRATION_CMD;
        frame.can_dlc = 8;
        memset(frame.data, 0, 8);
        frame.data[0] = command;

        int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
        return (nbytes == sizeof(struct can_frame));
    }

    bool wait_for_response(int timeout_ms = 5000) {
        auto start = std::chrono::steady_clock::now();

        while (!response_received) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);

            if (elapsed.count() > timeout_ms) {
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        return true;
    }

    bool check_response_status() {
        if (!response_received) return false;

        uint8_t echo_cmd = last_response.data[0];
        uint8_t status = last_response.data[1];

        std::cout << "[INFO] Response: Command=0x" << std::hex << (int)echo_cmd
                  << " Status=0x" << (int)status << std::dec;

        if (status == 0x00) {
            std::cout << " (SUCCESS)" << std::endl;
        } else {
            std::cout << " (FAILED)" << std::endl;
        }

        return (status == 0x00);
    }

    uint32_t get_raw_lux_value() {
        return last_raw_data.data[0] | (last_raw_data.data[1] << 8) | (last_raw_data.data[2] << 16);
    }

    uint8_t get_raw_range() {
        return last_raw_data.data[3];
    }

    uint32_t get_calibrated_lux_value() {
        // Parse 3-byte (24-bit) lux value from bytes 0-2
        return last_lux_data.data[0] |
               (last_lux_data.data[1] << 8) |
               (last_lux_data.data[2] << 16);
    }

    uint8_t get_sensor_status() {
        return last_lux_data.data[3];
    }

    uint8_t get_sequence_counter() {
        return last_lux_data.data[4];
    }

    uint8_t get_current_range() {
        return last_lux_data.data[5];
    }

    int recommend_range(uint16_t lux_value) {
        for (int i = 0; i < 4; i++) {
            if (lux_value >= range_info[i].optimal_min && lux_value <= range_info[i].optimal_max) {
                return i;
            }
        }

        for (int i = 0; i < 4; i++) {
            if (lux_value >= range_info[i].min_lux && lux_value <= range_info[i].max_lux) {
                return i;
            }
        }

        if (lux_value < range_info[0].min_lux) return 0;
        if (lux_value > range_info[3].max_lux) return 3;

        return -1;
    }

    void show_range_recommendation(uint16_t lux_value) {
        int recommended = recommend_range(lux_value);

        if (recommended >= 0) {
            std::cout << "[RECOMMENDATION] For " << lux_value << " lux, use: --reference-";
            switch (recommended) {
                case 0: std::cout << "low"; break;
                case 1: std::cout << "medium"; break;
                case 2: std::cout << "high"; break;
                case 3: std::cout << "veryhigh"; break;
            }
            std::cout << "=" << lux_value << std::endl;
            std::cout << "[INFO] " << range_names[recommended] << " range: " << range_info[recommended].description << std::endl;
        }
    }

    void show_current_readings() {
        std::cout << "[INFO] Entering calibration mode..." << std::endl;

        if (!send_command(CAL_CMD_ENTER)) {
            std::cerr << "[ERROR] Failed to send enter command" << std::endl;
            return;
        }

        if (!wait_for_response()) {
            std::cerr << "[ERROR] No response to enter command" << std::endl;
            return;
        }

        if (!check_response_status()) {
            std::cerr << "[ERROR] Enter command failed" << std::endl;
            return;
        }

        in_calibration_mode = true;
        std::cout << "[SUCCESS] Entered calibration mode" << std::endl;
        std::cout << "[INFO] Reading raw sensor data (10 samples)..." << std::endl;
        std::cout << "[INFO] Press Ctrl+C to exit cleanly" << std::endl;

        uint32_t sample_values[20];
        int valid_samples = 0;

        for (int i = 1; i <= 20; i++) {
            if (interrupt_requested) {
                std::cout << "[INFO] Interrupt received, exiting..." << std::endl;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1100));
            uint32_t lux = get_raw_lux_value();
            uint8_t range = get_raw_range();
            const char* range_name = (range < 4) ? range_names[range] : "UNKNOWN";

            std::cout << "[INFO] Sample " << i << ": " << lux << " lux (Range: "
                      << range_name << ")" << std::endl;

            if (lux > 0 && lux < 16777215 && valid_samples < 20) {
                sample_values[valid_samples++] = lux;
            }
        }

        if (valid_samples > 0) {
            uint64_t sum = 0;
            for (int i = 0; i < valid_samples; i++) {
                sum += sample_values[i];
            }
            uint32_t average = sum / valid_samples;

            std::cout << "\n[ANALYSIS] Average reading: " << average << " lux" << std::endl;
            show_range_recommendation(average);
        }

        std::cout << "\n[INFO] Exiting calibration mode..." << std::endl;
        send_command(CAL_CMD_EXIT);
        wait_for_response();
        check_response_status();
        in_calibration_mode = false;
    }

    void show_calibrated_readings() {
        std::cout << "[INFO] Monitoring calibrated sensor readings (20 samples)..." << std::endl;
        std::cout << "[INFO] Waiting for fresh CAN data..." << std::endl;

        uint8_t last_sequence = get_sequence_counter();
        bool got_fresh_data = false;

        for (int wait_count = 0; wait_count < 20 && !got_fresh_data; wait_count++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            uint8_t current_sequence = get_sequence_counter();
            if (current_sequence != last_sequence) {
                got_fresh_data = true;
                break;
            }
        }

        if (!got_fresh_data) {
            std::cerr << "[ERROR] No fresh CAN data received. Check if ESP32 is transmitting." << std::endl;
            return;
        }

        for (int i = 1; i <= 30; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1100));

            uint32_t calibrated_lux = get_calibrated_lux_value();
            uint8_t status = get_sensor_status();
            uint8_t sequence = get_sequence_counter();
            uint8_t range = get_current_range();

            const char* status_str = "UNKNOWN";
            switch (status) {
                case 0x00: status_str = "OK"; break;
                case 0x01: status_str = "ERROR"; break;
                case 0x02: status_str = "CALIBRATING"; break;
            }

            const char* range_name = (range < 4) ? range_names[range] : "UNKNOWN";

            std::cout << "[INFO] Sample " << i << ": " << calibrated_lux << " lux "
                      << "(status=" << status_str << ", seq=" << (int)sequence
                      << ", range=" << range_name << ")" << std::endl;
        }
    }

    bool validate_range_value(uint8_t range_cmd, uint16_t reference_lux, const char* range_name) {
        int range_idx = -1;
        switch (range_cmd) {
            case CAL_CMD_SET_REF_LOW: range_idx = 0; break;
            case CAL_CMD_SET_REF_MED: range_idx = 1; break;
            case CAL_CMD_SET_REF_HIGH: range_idx = 2; break;
            case CAL_CMD_SET_REF_VERYHIGH: range_idx = 3; break;
            default: return false;
        }

        const RangeInfo& info = range_info[range_idx];

        if (reference_lux < info.min_lux || reference_lux > info.max_lux) {
            std::cerr << "[ERROR] " << range_name << " range value " << reference_lux
                      << " lux is outside acceptable range (" << info.min_lux
                      << "-" << info.max_lux << " lux)" << std::endl;
            std::cerr << "[INFO] " << range_name << " range is for: " << info.description << std::endl;
            return false;
        }

        if (reference_lux < info.optimal_min || reference_lux > info.optimal_max) {
            std::cout << "[WARNING] " << reference_lux << " lux is acceptable but not optimal for "
                      << range_name << " range" << std::endl;
            std::cout << "[INFO] Optimal range: " << info.optimal_min << "-" << info.optimal_max
                      << " lux" << std::endl;

            int better_range = recommend_range(reference_lux);
            if (better_range != range_idx && better_range >= 0) {
                std::cout << "[SUGGESTION] Consider using --reference-";
                switch (better_range) {
                    case 0: std::cout << "low"; break;
                    case 1: std::cout << "medium"; break;
                    case 2: std::cout << "high"; break;
                    case 3: std::cout << "veryhigh"; break;
                }
                std::cout << "=" << reference_lux << " instead" << std::endl;
            }
        }

        return true;
    }

    void calibrate_range(uint8_t range_cmd, const char* range_name, uint16_t reference_lux, bool verify = false) {
        if (!validate_range_value(range_cmd, reference_lux, range_name)) {
            return;
        }

        std::cout << "[INFO] Starting " << range_name << " range calibration with reference: "
                  << reference_lux << " lux" << std::endl;

        std::cout << "[INFO] Entering calibration mode..." << std::endl;
        if (!send_command(CAL_CMD_ENTER) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to enter calibration mode" << std::endl;
            return;
        }

        in_calibration_mode = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        uint32_t current_lux = get_raw_lux_value();
        uint8_t current_range = get_raw_range();

        std::cout << "[INFO] Current sensor reading: " << current_lux << " lux (Range: "
                  << ((current_range < 4) ? range_names[current_range] : "UNKNOWN") << ")" << std::endl;
        std::cout << "[INFO] Reference lux meter: " << reference_lux << " lux" << std::endl;

        int32_t expected_offset = reference_lux - current_lux;
        std::cout << "[INFO] Expected offset: " << expected_offset << " lux" << std::endl;

        uint8_t ref_data[2] = {
            static_cast<uint8_t>(reference_lux & 0xFF),
            static_cast<uint8_t>((reference_lux >> 8) & 0xFF)
        };

        std::cout << "[INFO] Setting " << range_name << " range reference value..." << std::endl;
        if (!send_command(range_cmd, ref_data, 2) ||
            !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to set " << range_name << " range reference value" << std::endl;
            send_command(CAL_CMD_EXIT);
            in_calibration_mode = false;
            return;
        }

        std::cout << "[INFO] Saving calibration..." << std::endl;
        if (!send_command(CAL_CMD_SAVE) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to save calibration" << std::endl;
            send_command(CAL_CMD_EXIT);
            in_calibration_mode = false;
            return;
        }

        std::cout << "[INFO] Exiting calibration mode..." << std::endl;
        if (!send_command(CAL_CMD_EXIT) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to exit calibration mode" << std::endl;
            return;
        }

        in_calibration_mode = false;

        std::cout << "[SUCCESS] " << range_name << " range calibration completed!" << std::endl;
        std::cout << "[INFO] Calculated offset: " << expected_offset << " lux" << std::endl;

        if (verify) {
            std::cout << "[INFO] Verifying calibration..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            show_calibrated_readings();
        }
    }

    void calibrate(uint16_t reference_lux, bool verify = false) {
        calibrate_range(CAL_CMD_SET_REFERENCE, "MEDIUM", reference_lux, verify);
    }

    void get_all_offsets() {
        std::cout << "[INFO] Getting all calibration offsets..." << std::endl;

        if (!send_command(CAL_CMD_GET_ALL_OFFSETS) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to get offsets" << std::endl;
            return;
        }

        float first_offset;
        memcpy(&first_offset, &last_response.data[2], sizeof(float));
        std::cout << "[INFO] First range (LOW) calibration offset: " << first_offset << " lux" << std::endl;
        std::cout << "[INFO] Check ESP32 console logs for all 4 range offsets" << std::endl;
    }

    void get_offset() {
        std::cout << "[INFO] Getting current offset (legacy - medium range only)..." << std::endl;

        if (!send_command(CAL_CMD_GET_OFFSET) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to get offset" << std::endl;
            return;
        }

        float offset;
        memcpy(&offset, &last_response.data[2], sizeof(float));
        std::cout << "[INFO] Current calibration offset (medium range): " << offset << " lux" << std::endl;
    }

    void reset_calibration() {
        std::cout << "[INFO] Resetting ALL calibration ranges to factory defaults..." << std::endl;

        if (!send_command(CAL_CMD_RESET) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to reset calibration" << std::endl;
            return;
        }

        std::cout << "[SUCCESS] All calibration ranges reset to factory defaults (offset = 0.0 lux)" << std::endl;
        std::cout << "[INFO] Sensor will now report uncalibrated raw values for all ranges" << std::endl;
    }

    void calibration_wizard() {
        std::cout << "=== 4-Range Calibration Wizard ===" << std::endl;
        std::cout << "This will guide you through calibrating all 4 light ranges." << std::endl;
        std::cout << "You'll need to adjust lighting and use your reference lux meter." << std::endl;
        std::cout << std::endl;

        for (int i = 0; i < 4; i++) {
            std::cout << "Range " << (i+1) << " (" << range_names[i] << "): "
                      << range_info[i].description << std::endl;
            std::cout << "  Optimal: " << range_info[i].optimal_min << "-"
                      << range_info[i].optimal_max << " lux" << std::endl;
        }

        std::cout << "\nPress Enter to start, or Ctrl+C to abort..." << std::endl;
        std::cin.get();

        std::cout << "\nStep 1: LOW Range (" << range_info[0].optimal_min << "-"
                  << range_info[0].optimal_max << " lux)" << std::endl;
        std::cout << "- " << range_info[0].description << std::endl;
        std::cout << "- Measure with your reference lux meter" << std::endl;
        std::cout << "- Enter the measured value and press Enter: ";

        uint16_t low_ref;
        std::cin >> low_ref;
        calibrate_range(0x07, "LOW", low_ref);

        std::cout << "\nStep 2: MEDIUM Range (" << range_info[1].optimal_min << "-"
                  << range_info[1].optimal_max << " lux)" << std::endl;
        std::cout << "- " << range_info[1].description << std::endl;
        std::cout << "- Enter the measured value and press Enter: ";

        uint16_t med_ref;
        std::cin >> med_ref;
        calibrate_range(0x08, "MEDIUM", med_ref);

        std::cout << "\nStep 3: HIGH Range (" << range_info[2].optimal_min << "-"
                  << range_info[2].optimal_max << " lux)" << std::endl;
        std::cout << "- " << range_info[2].description << std::endl;
        std::cout << "- Enter the measured value and press Enter: ";

        uint16_t high_ref;
        std::cin >> high_ref;
        calibrate_range(0x09, "HIGH", high_ref);

        std::cout << "\nStep 4: VERY HIGH Range (" << range_info[3].optimal_min << "-"
                  << range_info[3].optimal_max << " lux)" << std::endl;
        std::cout << "- " << range_info[3].description << std::endl;
        std::cout << "- Enter the measured value and press Enter: ";

        uint16_t vhigh_ref;
        std::cin >> vhigh_ref;
        calibrate_range(0x0A, "VERY_HIGH", vhigh_ref);

        std::cout << "\n=== Calibration Complete! ===" << std::endl;
        std::cout << "All 4 ranges have been calibrated." << std::endl;
        std::cout << "The sensor will now automatically switch between ranges." << std::endl;

        std::cout << "\nVerification readings:" << std::endl;
        show_calibrated_readings();
    }
};

void print_usage(const char* program_name) {
    std::cout << "ESP32-C6 Lux Sensor 4-Range Calibration Tool (C++)" << std::endl;
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << "4-Range Calibration Options:" << std::endl;
    std::cout << "  --reference-low=VALUE      Calibrate LOW range (1-50 lux, optimal: 5-35)" << std::endl;
    std::cout << "  --reference-medium=VALUE   Calibrate MEDIUM range (10-350 lux, optimal: 20-250)" << std::endl;
    std::cout << "  --reference-high=VALUE     Calibrate HIGH range (100-1200 lux, optimal: 200-800)" << std::endl;
    std::cout << "  --reference-veryhigh=VALUE Calibrate VERY HIGH range (500-5000 lux, optimal: 800-2000)" << std::endl;
    std::cout << "  --wizard                   Interactive 4-step calibration wizard" << std::endl;
    std::cout << "  --get-all-offsets          Show all 4 range calibration offsets" << std::endl;
    std::cout << std::endl;
    std::cout << "Legacy Options (for backward compatibility):" << std::endl;
    std::cout << "  --reference=VALUE          Calibrate using reference lux meter value (medium range)" << std::endl;
    std::cout << "  --get-offset               Show current calibration offset (medium range only)" << std::endl;
    std::cout << std::endl;
    std::cout << "General Options:" << std::endl;
    std::cout << "  --verify                   Verify calibration after setting" << std::endl;
    std::cout << "  --show-current             Display raw sensor readings with range recommendations" << std::endl;
    std::cout << "  --show-calibrated          Display calibrated sensor readings (normal mode)" << std::endl;
    std::cout << "  --reset-calibration        Reset ALL calibration ranges to factory defaults" << std::endl;
    std::cout << "  --help                     Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " --show-current          # Shows readings and recommends correct range" << std::endl;
    std::cout << "  " << program_name << " --reference-high=1118   # Correct for your 1118 lux case" << std::endl;
    std::cout << "  " << program_name << " --wizard               # Guided calibration" << std::endl;
    std::cout << "  " << program_name << " --reset-calibration    # Start fresh" << std::endl;
    std::cout << std::endl;
    std::cout << "Range Selection Guide:" << std::endl;
    std::cout << "  LOW (5-35 lux):      Very dim (night, parking lights)" << std::endl;
    std::cout << "  MEDIUM (20-250 lux): Normal indoor lighting" << std::endl;
    std::cout << "  HIGH (200-800 lux):  Bright indoor to daylight" << std::endl;
    std::cout << "  V_HIGH (800+ lux):   Direct sunlight, very bright conditions" << std::endl;
}

void signal_handler(int signum) {
    std::cout << "\n[INFO] Received signal " << signum << ", cleaning up..." << std::endl;

    if (g_calibrator) {
        g_calibrator->interrupt_requested = true;
        g_calibrator->cleanup();
    }

    exit(signum);
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    CANCalibrator calibrator;

    if (!calibrator.initialize()) {
        std::cerr << "[ERROR] Failed to initialize CAN interface" << std::endl;
        std::cerr << "[INFO] Make sure can0 is up: sudo ip link set can0 type can bitrate 500000 && sudo ip link set up can0" << std::endl;
        return 1;
    }

    std::thread listener(&CANCalibrator::listen_thread, &calibrator);

    bool show_current = false;
    bool show_calibrated = false;
    bool get_offset = false;
    bool get_all_offsets = false;
    bool reset_calibration = false;
    bool verify = false;
    bool wizard = false;
    uint16_t reference_lux = 0;
    uint16_t reference_low = 0;
    uint16_t reference_medium = 0;
    uint16_t reference_high = 0;
    uint16_t reference_veryhigh = 0;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--show-current") {
            show_current = true;
        } else if (arg == "--show-calibrated") {
            show_calibrated = true;
        } else if (arg == "--get-offset") {
            get_offset = true;
        } else if (arg == "--get-all-offsets") {
            get_all_offsets = true;
        } else if (arg == "--reset-calibration") {
            reset_calibration = true;
        } else if (arg == "--verify") {
            verify = true;
        } else if (arg == "--wizard") {
            wizard = true;
        } else if (arg.substr(0, 12) == "--reference=") {
            reference_lux = std::stoi(arg.substr(12));
        } else if (arg.substr(0, 16) == "--reference-low=") {
            reference_low = std::stoi(arg.substr(16));
        } else if (arg.substr(0, 19) == "--reference-medium=") {
            reference_medium = std::stoi(arg.substr(19));
        } else if (arg.substr(0, 17) == "--reference-high=") {
            reference_high = std::stoi(arg.substr(17));
        } else if (arg.substr(0, 21) == "--reference-veryhigh=") {
            reference_veryhigh = std::stoi(arg.substr(21));
        } else if (arg == "--help") {
            print_usage(argv[0]);
            calibrator.running = false;
            listener.join();
            return 0;
        } else {
            std::cerr << "[ERROR] Unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            calibrator.running = false;
            listener.join();
            return 1;
        }
    }

    // Execute commands
    if (wizard) {
        calibrator.calibration_wizard();
    } else if (show_current) {
        calibrator.show_current_readings();
    } else if (show_calibrated) {
        calibrator.show_calibrated_readings();
    } else if (get_offset) {
        calibrator.get_offset();
    } else if (get_all_offsets) {
        calibrator.get_all_offsets();
    } else if (reset_calibration) {
        calibrator.reset_calibration();
    } else if (reference_low > 0) {
        calibrator.calibrate_range(0x07, "LOW", reference_low, verify);
    } else if (reference_medium > 0) {
        calibrator.calibrate_range(0x08, "MEDIUM", reference_medium, verify);
    } else if (reference_high > 0) {
        calibrator.calibrate_range(0x09, "HIGH", reference_high, verify);
    } else if (reference_veryhigh > 0) {
        calibrator.calibrate_range(0x0A, "VERY_HIGH", reference_veryhigh, verify);
    } else if (reference_lux > 0) {
        // Legacy calibration
        if (reference_lux < 1 || reference_lux > 10000) {
            std::cerr << "[ERROR] Reference value must be between 1 and 10000 lux" << std::endl;
            calibrator.running = false;
            listener.join();
            return 1;
        }
        calibrator.calibrate(reference_lux, verify);
    } else {
        std::cerr << "[ERROR] No action specified" << std::endl;
        print_usage(argv[0]);
        calibrator.running = false;
        listener.join();
        return 1;
    }

    // Clean shutdown
    calibrator.running = false;
    listener.join();

    return 0;
}
