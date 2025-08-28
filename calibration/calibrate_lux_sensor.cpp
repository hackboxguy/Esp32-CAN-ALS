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

class CANCalibrator {
private:
    int can_socket;
    std::atomic<bool> response_received{false};
    struct can_frame last_response;
    struct can_frame last_raw_data;
    struct can_frame last_lux_data;  // Added for normal lux data messages

public:
    std::atomic<bool> running{true};  // Made public for main() access

    // CAN IDs
    static constexpr canid_t ID_CALIBRATION_CMD = 0x0A3;
    static constexpr canid_t ID_CALIBRATION_RESP = 0x0A4;
    static constexpr canid_t ID_RAW_SENSOR_DATA = 0x0A5;

    // Commands
    static constexpr uint8_t CAL_CMD_ENTER = 0x01;
    static constexpr uint8_t CAL_CMD_SET_REFERENCE = 0x02;
    static constexpr uint8_t CAL_CMD_SAVE = 0x03;
    static constexpr uint8_t CAL_CMD_EXIT = 0x04;
    static constexpr uint8_t CAL_CMD_GET_OFFSET = 0x05;

public:
    CANCalibrator() : can_socket(-1) {}

    ~CANCalibrator() {
        if (can_socket >= 0) {
            close(can_socket);
        }
    }

    bool initialize(const char* interface = "can0") {
        can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket < 0) {
            std::cerr << "Error: Cannot create CAN socket" << std::endl;
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
            std::cerr << "Error: Cannot bind to CAN interface " << interface << std::endl;
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
            } else if (frame.can_id == 0x0A2) {  // Normal lux data messages
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
                  << " Status=0x" << (int)status << std::dec << std::endl;

        return (status == 0x00);
    }

    uint16_t get_raw_lux_value() {
        // Extract little-endian 16-bit value from raw data
        return last_raw_data.data[0] | (last_raw_data.data[1] << 8);
    }

    uint16_t get_calibrated_lux_value() {
        // Extract little-endian 16-bit value from normal lux data (0xA2 messages)
        return last_lux_data.data[0] | (last_lux_data.data[1] << 8);
    }

    uint8_t get_sensor_status() {
        // Extract sensor status from normal lux data
        return last_lux_data.data[2];
    }

    uint8_t get_sequence_counter() {
        // Extract sequence counter from normal lux data
        return last_lux_data.data[3];
    }

    void show_calibrated_readings() {
        std::cout << "[INFO] Monitoring calibrated sensor readings (10 samples)..." << std::endl;
        std::cout << "[INFO] Press Ctrl+C to stop" << std::endl;

        for (int i = 1; i <= 10; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1100)); // Wait for next sample

            uint16_t calibrated_lux = get_calibrated_lux_value();
            uint8_t status = get_sensor_status();
            uint8_t sequence = get_sequence_counter();

            const char* status_str = "UNKNOWN";
            switch (status) {
                case 0x00: status_str = "OK"; break;
                case 0x01: status_str = "ERROR"; break;
                case 0x02: status_str = "CALIBRATING"; break;
            }

            std::cout << "[INFO] Sample " << i << ": " << calibrated_lux << " lux "
                      << "(status=" << status_str << ", seq=" << (int)sequence << ")" << std::endl;
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

        std::cout << "[SUCCESS] Entered calibration mode" << std::endl;
        std::cout << "[INFO] Reading sensor data (10 samples)..." << std::endl;

        for (int i = 1; i <= 10; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1100)); // Wait for next sample
            uint16_t lux = get_raw_lux_value();
            std::cout << "[INFO] Sample " << i << ": " << lux << " lux" << std::endl;
        }

        // Exit calibration mode
        std::cout << "[INFO] Exiting calibration mode..." << std::endl;
        send_command(CAL_CMD_EXIT);
        wait_for_response();
        check_response_status();
    }

    void calibrate(uint16_t reference_lux, bool verify = false) {
        std::cout << "[INFO] Starting calibration with reference: " << reference_lux << " lux" << std::endl;

        // Enter calibration mode
        std::cout << "[INFO] Entering calibration mode..." << std::endl;
        if (!send_command(CAL_CMD_ENTER) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to enter calibration mode" << std::endl;
            return;
        }

        // Wait for sensor data and read current value
        std::this_thread::sleep_for(std::chrono::milliseconds(1100));
        uint16_t current_lux = get_raw_lux_value();
        std::cout << "[INFO] Current sensor reading: " << current_lux << " lux" << std::endl;
        std::cout << "[INFO] Reference lux meter: " << reference_lux << " lux" << std::endl;

        // Send reference value (little-endian)
        uint8_t ref_data[2] = {
            static_cast<uint8_t>(reference_lux & 0xFF),
            static_cast<uint8_t>((reference_lux >> 8) & 0xFF)
        };

        std::cout << "[INFO] Setting reference value..." << std::endl;
        if (!send_command(CAL_CMD_SET_REFERENCE, ref_data, 2) ||
            !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to set reference value" << std::endl;
            return;
        }

        // Save calibration
        std::cout << "[INFO] Saving calibration..." << std::endl;
        if (!send_command(CAL_CMD_SAVE) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to save calibration" << std::endl;
            return;
        }

        // Exit calibration mode
        std::cout << "[INFO] Exiting calibration mode..." << std::endl;
        if (!send_command(CAL_CMD_EXIT) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to exit calibration mode" << std::endl;
            return;
        }

        int16_t offset = reference_lux - current_lux;
        std::cout << "[SUCCESS] Calibration completed!" << std::endl;
        std::cout << "[INFO] Calculated offset: " << offset << " lux" << std::endl;

        if (verify) {
            std::cout << "[INFO] Verifying calibration..." << std::endl;
            // Verification would monitor normal CAN messages (ID 0A2)
            // This is left as an exercise since it would need different message parsing
        }
    }

    void get_offset() {
        std::cout << "[INFO] Getting current offset..." << std::endl;

        if (!send_command(CAL_CMD_GET_OFFSET) || !wait_for_response() || !check_response_status()) {
            std::cerr << "[ERROR] Failed to get offset" << std::endl;
            return;
        }

        // Extract float from response data (bytes 2-5)
        float offset;
        memcpy(&offset, &last_response.data[2], sizeof(float));
        std::cout << "[INFO] Current calibration offset: " << offset << " lux" << std::endl;
    }
};

void print_usage(const char* program_name) {
    std::cout << "ESP32-C6 Lux Sensor Calibration Tool (C++)" << std::endl;
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " [OPTIONS]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --reference=VALUE    Calibrate using reference lux meter value" << std::endl;
    std::cout << "  --verify            Verify calibration after setting" << std::endl;
    std::cout << "  --show-current      Display raw sensor readings (calibration mode)" << std::endl;
    std::cout << "  --show-calibrated   Display calibrated sensor readings (normal mode)" << std::endl;
    std::cout << "  --get-offset        Show current calibration offset" << std::endl;
    std::cout << "  --help              Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " --reference=102" << std::endl;
    std::cout << "  " << program_name << " --reference=250 --verify" << std::endl;
    std::cout << "  " << program_name << " --show-current" << std::endl;
    std::cout << "  " << program_name << " --show-calibrated" << std::endl;
}

int main(int argc, char* argv[]) {
    CANCalibrator calibrator;

    if (!calibrator.initialize()) {
        std::cerr << "[ERROR] Failed to initialize CAN interface" << std::endl;
        std::cerr << "[INFO] Make sure can0 is up: sudo ip link set can0 type can bitrate 500000 && sudo ip link set up can0" << std::endl;
        return 1;
    }

    // Start listening thread
    std::thread listener(&CANCalibrator::listen_thread, &calibrator);

    // Parse arguments
    bool show_current = false;
    bool show_calibrated = false;
    bool get_offset = false;
    bool verify = false;
    uint16_t reference_lux = 0;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--show-current") {
            show_current = true;
        } else if (arg == "--show-calibrated") {
            show_calibrated = true;
        } else if (arg == "--get-offset") {
            get_offset = true;
        } else if (arg == "--verify") {
            verify = true;
        } else if (arg.substr(0, 12) == "--reference=") {
            reference_lux = std::stoi(arg.substr(12));
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "[ERROR] Unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    // Execute commands
    if (show_current) {
        calibrator.show_current_readings();
    } else if (show_calibrated) {
        calibrator.show_calibrated_readings();
    } else if (get_offset) {
        calibrator.get_offset();
    } else if (reference_lux > 0) {
        if (reference_lux < 1 || reference_lux > 10000) {
            std::cerr << "[ERROR] Reference value must be between 1 and 10000 lux" << std::endl;
            return 1;
        }
        calibrator.calibrate(reference_lux, verify);
    } else {
        std::cerr << "[ERROR] No action specified" << std::endl;
        print_usage(argv[0]);
        // Clean shutdown even when no action taken
        calibrator.running = false;
        listener.join();
        return 1;
    }

    // Clean shutdown
    calibrator.running = false;
    listener.join();

    return 0;
}
