/**
 * can-sensor-tool.cpp - CAN Sensor Node Management Tool
 *
 * A CLI utility for managing ESP32 CAN sensor nodes.
 * Supports monitoring, control, configuration, and diagnostics.
 *
 * Requirements: Linux with SocketCAN support
 * Build: cmake .. && make
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cstdarg>
#include <string>
#include <vector>
#include <optional>
#include <chrono>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <errno.h>

/* ============================================================================
 * Version Information
 * ========================================================================== */

#ifndef TOOL_VERSION_MAJOR
#define TOOL_VERSION_MAJOR 1
#endif
#ifndef TOOL_VERSION_MINOR
#define TOOL_VERSION_MINOR 0
#endif
#ifndef TOOL_VERSION_PATCH
#define TOOL_VERSION_PATCH 0
#endif

/* ============================================================================
 * CAN Protocol Constants
 * ========================================================================== */

namespace can_protocol {

// Node addressing: each node has 16 message IDs (0x10 spacing)
constexpr uint32_t NODE_ADDR_SPACING = 0x10;
constexpr uint32_t BASE_ADDR_NODE_0 = 0x0A0;
constexpr int MAX_NODE_ID = 5;

// Message offsets from node base address
enum class MsgOffset : uint8_t {
    STOP           = 0x00,  // Stop transmission
    START          = 0x01,  // Start transmission
    ALS_DATA       = 0x02,  // Ambient light sensor data
    ENV_DATA       = 0x03,  // Environmental data (T/H/P)
    AIR_QUALITY    = 0x04,  // Air quality (IAQ/CO2/VOC)
    // 0x05-0x07 reserved for future sensors
    SHUTDOWN       = 0x08,  // Graceful shutdown
    REBOOT         = 0x09,  // Reboot
    FACTORY_RESET  = 0x0A,  // Factory reset
    SET_NODE_ID    = 0x0B,  // Set node ID
    GET_INFO       = 0x0C,  // Request device info
    INFO_RESPONSE  = 0x0D,  // Device info response
    PING           = 0x0E,  // Discovery ping
    PONG           = 0x0F,  // Discovery response
};

// Calculate CAN ID for a given node and message type
inline uint32_t make_can_id(int node_id, MsgOffset offset) {
    return BASE_ADDR_NODE_0 + (node_id * NODE_ADDR_SPACING) + static_cast<uint8_t>(offset);
}

// Extract node ID from a CAN ID (returns -1 if not in valid range)
inline int extract_node_id(uint32_t can_id) {
    if (can_id < BASE_ADDR_NODE_0 || can_id > (BASE_ADDR_NODE_0 + (MAX_NODE_ID + 1) * NODE_ADDR_SPACING - 1)) {
        return -1;
    }
    return (can_id - BASE_ADDR_NODE_0) / NODE_ADDR_SPACING;
}

// Extract message offset from a CAN ID
inline MsgOffset extract_offset(uint32_t can_id) {
    return static_cast<MsgOffset>((can_id - BASE_ADDR_NODE_0) % NODE_ADDR_SPACING);
}

} // namespace can_protocol

/* ============================================================================
 * Output Format
 * ========================================================================== */

enum class OutputFormat {
    HUMAN,
    JSON,
    CSV
};

/* ============================================================================
 * Command Types
 * ========================================================================== */

enum class Command {
    NONE,
    HELP,
    VERSION,
    START,
    STOP,
    SHUTDOWN,
    REBOOT,
    FACTORY_RESET,
    MONITOR,
    SET_NODE_ID,
    INFO,
    DISCOVER,
};

/* ============================================================================
 * Configuration Structure
 * ========================================================================== */

struct Config {
    // Global options
    std::string interface = "can0";
    int speed = 500000;  // For reference only (interface must be pre-configured)
    int node_id = 0;
    bool quiet = false;

    // Command
    Command command = Command::NONE;

    // Monitor options
    OutputFormat format = OutputFormat::HUMAN;
    int count = 0;      // 0 = unlimited
    int timeout = 0;    // 0 = unlimited (seconds)

    // Set node ID option
    int new_node_id = -1;
};

/* ============================================================================
 * CAN Socket Class
 * ========================================================================== */

class CanSocket {
public:
    CanSocket() : fd_(-1) {}
    ~CanSocket() { close(); }

    // Non-copyable
    CanSocket(const CanSocket&) = delete;
    CanSocket& operator=(const CanSocket&) = delete;

    // Open CAN socket on specified interface
    bool open(const std::string& interface) {
        // Create socket
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) {
            perror("socket");
            return false;
        }

        // Get interface index
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            perror("ioctl SIOCGIFINDEX");
            close();
            return false;
        }

        // Bind to interface
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("bind");
            close();
            return false;
        }

        interface_ = interface;
        return true;
    }

    // Close socket
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    // Check if socket is open
    bool is_open() const { return fd_ >= 0; }

    // Send CAN frame
    bool send(uint32_t can_id, const uint8_t* data, uint8_t len) {
        if (fd_ < 0) return false;
        if (len > CAN_MAX_DLEN) len = CAN_MAX_DLEN;

        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = can_id;
        frame.can_dlc = len;
        if (data && len > 0) {
            std::memcpy(frame.data, data, len);
        }

        ssize_t nbytes = write(fd_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            perror("write");
            return false;
        }
        return true;
    }

    // Send empty CAN frame (command only)
    bool send(uint32_t can_id) {
        return send(can_id, nullptr, 0);
    }

    // Receive CAN frame with timeout
    // Returns true if frame received, false on timeout or error
    // timeout_ms: -1 = block forever, 0 = non-blocking, >0 = timeout in ms
    bool receive(struct can_frame& frame, int timeout_ms = -1) {
        if (fd_ < 0) return false;

        struct pollfd pfd;
        pfd.fd = fd_;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, timeout_ms);
        if (ret < 0) {
            if (errno != EINTR) {
                perror("poll");
            }
            return false;
        }
        if (ret == 0) {
            // Timeout
            return false;
        }

        ssize_t nbytes = read(fd_, &frame, sizeof(frame));
        if (nbytes < 0) {
            perror("read");
            return false;
        }
        if (nbytes < static_cast<ssize_t>(sizeof(frame))) {
            fprintf(stderr, "Incomplete CAN frame received\n");
            return false;
        }

        return true;
    }

    // Get file descriptor (for advanced use)
    int fd() const { return fd_; }

    // Get interface name
    const std::string& interface() const { return interface_; }

private:
    int fd_;
    std::string interface_;
};

/* ============================================================================
 * Utility Functions
 * ========================================================================== */

// Print message unless quiet mode
static bool g_quiet = false;

void info(const char* fmt, ...) {
    if (g_quiet) return;
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
}

// Parse integer with validation
std::optional<int> parse_int(const char* str) {
    char* end;
    long val = strtol(str, &end, 10);
    if (*end != '\0' || end == str) {
        return std::nullopt;
    }
    return static_cast<int>(val);
}

/* ============================================================================
 * Help and Version
 * ========================================================================== */

void print_version() {
    printf("can-sensor-tool version %d.%d.%d\n",
           TOOL_VERSION_MAJOR, TOOL_VERSION_MINOR, TOOL_VERSION_PATCH);
}

void print_usage(const char* prog_name) {
    printf("Usage: %s [OPTIONS] <COMMAND> [ARGS]\n", prog_name);
    printf("\n");
    printf("CAN Sensor Node Management Tool\n");
    printf("\n");
    printf("OPTIONS:\n");
    printf("  --interface=<iface>   CAN interface (default: can0)\n");
    printf("  --speed=<bps>         CAN speed in bps (default: 500000, for reference)\n");
    printf("  --node-id=<id>        Target node ID 0-%d (default: 0)\n", can_protocol::MAX_NODE_ID);
    printf("  --quiet               Suppress status messages\n");
    printf("  --help                Show this help message\n");
    printf("  --version             Show version information\n");
    printf("\n");
    printf("COMMANDS:\n");
    printf("  Control:\n");
    printf("    start               Start sensor transmission\n");
    printf("    stop                Stop sensor transmission\n");
    printf("    shutdown            Graceful shutdown (save state, stop TX)\n");
    printf("    reboot              Save state and reboot device\n");
    printf("    factory-reset       Clear calibration and reboot\n");
    printf("\n");
    printf("  Monitoring:\n");
    printf("    monitor             Display live sensor data\n");
    printf("      --format=<fmt>    Output format: human, json, csv (default: human)\n");
    printf("      --count=<n>       Stop after n messages (default: unlimited)\n");
    printf("      --timeout=<sec>   Stop after n seconds (default: unlimited)\n");
    printf("\n");
    printf("  Configuration:\n");
    printf("    set-node-id <id>    Set device node ID (triggers reboot)\n");
    printf("    info                Query device information\n");
    printf("    discover            Scan bus for all sensor nodes\n");
    printf("\n");
    printf("EXAMPLES:\n");
    printf("  %s monitor                        # Monitor node 0\n", prog_name);
    printf("  %s --node-id=1 reboot             # Reboot node 1\n", prog_name);
    printf("  %s --interface=can1 discover      # Discover nodes on can1\n", prog_name);
    printf("  %s monitor --format=json --quiet  # JSON output for scripting\n", prog_name);
    printf("\n");
}

/* ============================================================================
 * Command-Line Parsing
 * ========================================================================== */

// Parse --option=value style argument
// Returns value if matched, empty string if option matches but no value, nullopt if no match
std::optional<std::string> parse_option(const char* arg, const char* option_name) {
    size_t opt_len = strlen(option_name);

    // Check for --option=value
    if (strncmp(arg, option_name, opt_len) == 0) {
        if (arg[opt_len] == '=') {
            return std::string(arg + opt_len + 1);
        }
        if (arg[opt_len] == '\0') {
            return std::string("");  // Flag without value
        }
    }
    return std::nullopt;
}

bool parse_args(int argc, char* argv[], Config& config) {
    std::vector<std::string> positional;

    for (int i = 1; i < argc; i++) {
        const char* arg = argv[i];

        // Help
        if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
            config.command = Command::HELP;
            return true;
        }

        // Version
        if (strcmp(arg, "--version") == 0 || strcmp(arg, "-v") == 0) {
            config.command = Command::VERSION;
            return true;
        }

        // Quiet
        if (strcmp(arg, "--quiet") == 0 || strcmp(arg, "-q") == 0) {
            config.quiet = true;
            continue;
        }

        // --interface=
        if (auto val = parse_option(arg, "--interface")) {
            if (val->empty()) {
                fprintf(stderr, "Error: --interface requires a value\n");
                return false;
            }
            config.interface = *val;
            continue;
        }

        // --speed=
        if (auto val = parse_option(arg, "--speed")) {
            if (val->empty()) {
                fprintf(stderr, "Error: --speed requires a value\n");
                return false;
            }
            auto speed = parse_int(val->c_str());
            if (!speed) {
                fprintf(stderr, "Error: Invalid speed value: %s\n", val->c_str());
                return false;
            }
            config.speed = *speed;
            continue;
        }

        // --node-id=
        if (auto val = parse_option(arg, "--node-id")) {
            if (val->empty()) {
                fprintf(stderr, "Error: --node-id requires a value\n");
                return false;
            }
            auto node = parse_int(val->c_str());
            if (!node || *node < 0 || *node > can_protocol::MAX_NODE_ID) {
                fprintf(stderr, "Error: Invalid node-id (must be 0-%d): %s\n",
                        can_protocol::MAX_NODE_ID, val->c_str());
                return false;
            }
            config.node_id = *node;
            continue;
        }

        // --format=
        if (auto val = parse_option(arg, "--format")) {
            if (*val == "human") {
                config.format = OutputFormat::HUMAN;
            } else if (*val == "json") {
                config.format = OutputFormat::JSON;
            } else if (*val == "csv") {
                config.format = OutputFormat::CSV;
            } else {
                fprintf(stderr, "Error: Invalid format (use human, json, or csv): %s\n", val->c_str());
                return false;
            }
            continue;
        }

        // --count=
        if (auto val = parse_option(arg, "--count")) {
            auto count = parse_int(val->c_str());
            if (!count || *count < 0) {
                fprintf(stderr, "Error: Invalid count value: %s\n", val->c_str());
                return false;
            }
            config.count = *count;
            continue;
        }

        // --timeout=
        if (auto val = parse_option(arg, "--timeout")) {
            auto timeout = parse_int(val->c_str());
            if (!timeout || *timeout < 0) {
                fprintf(stderr, "Error: Invalid timeout value: %s\n", val->c_str());
                return false;
            }
            config.timeout = *timeout;
            continue;
        }

        // Unknown option
        if (arg[0] == '-') {
            fprintf(stderr, "Error: Unknown option: %s\n", arg);
            return false;
        }

        // Positional argument
        positional.push_back(arg);
    }

    // Parse command from positional arguments
    if (positional.empty()) {
        fprintf(stderr, "Error: No command specified. Use --help for usage.\n");
        return false;
    }

    const std::string& cmd = positional[0];

    if (cmd == "start") {
        config.command = Command::START;
    } else if (cmd == "stop") {
        config.command = Command::STOP;
    } else if (cmd == "shutdown") {
        config.command = Command::SHUTDOWN;
    } else if (cmd == "reboot") {
        config.command = Command::REBOOT;
    } else if (cmd == "factory-reset") {
        config.command = Command::FACTORY_RESET;
    } else if (cmd == "monitor") {
        config.command = Command::MONITOR;
    } else if (cmd == "set-node-id") {
        config.command = Command::SET_NODE_ID;
        if (positional.size() < 2) {
            fprintf(stderr, "Error: set-node-id requires a node ID argument\n");
            return false;
        }
        auto new_id = parse_int(positional[1].c_str());
        if (!new_id || *new_id < 0 || *new_id > can_protocol::MAX_NODE_ID) {
            fprintf(stderr, "Error: Invalid new node ID (must be 0-%d): %s\n",
                    can_protocol::MAX_NODE_ID, positional[1].c_str());
            return false;
        }
        config.new_node_id = *new_id;
    } else if (cmd == "info") {
        config.command = Command::INFO;
    } else if (cmd == "discover") {
        config.command = Command::DISCOVER;
    } else {
        fprintf(stderr, "Error: Unknown command: %s\n", cmd.c_str());
        return false;
    }

    return true;
}

/* ============================================================================
 * Command Implementations (Placeholders for Phase 2+)
 * ========================================================================== */

int cmd_start(CanSocket& can, const Config& config) {
    uint32_t can_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::START);
    info("Sending START to node %d (CAN ID 0x%03X)...\n", config.node_id, can_id);

    if (!can.send(can_id)) {
        fprintf(stderr, "Error: Failed to send START command\n");
        return 1;
    }

    info("START command sent successfully\n");
    return 0;
}

int cmd_stop(CanSocket& can, const Config& config) {
    uint32_t can_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::STOP);
    info("Sending STOP to node %d (CAN ID 0x%03X)...\n", config.node_id, can_id);

    if (!can.send(can_id)) {
        fprintf(stderr, "Error: Failed to send STOP command\n");
        return 1;
    }

    info("STOP command sent successfully\n");
    return 0;
}

int cmd_shutdown(CanSocket& can, const Config& config) {
    uint32_t can_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::SHUTDOWN);
    info("Sending SHUTDOWN to node %d (CAN ID 0x%03X)...\n", config.node_id, can_id);

    if (!can.send(can_id)) {
        fprintf(stderr, "Error: Failed to send SHUTDOWN command\n");
        return 1;
    }

    info("SHUTDOWN command sent successfully\n");
    return 0;
}

int cmd_reboot(CanSocket& can, const Config& config) {
    uint32_t can_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::REBOOT);
    info("Sending REBOOT to node %d (CAN ID 0x%03X)...\n", config.node_id, can_id);

    if (!can.send(can_id)) {
        fprintf(stderr, "Error: Failed to send REBOOT command\n");
        return 1;
    }

    info("REBOOT command sent successfully\n");
    return 0;
}

int cmd_factory_reset(CanSocket& can, const Config& config) {
    uint32_t can_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::FACTORY_RESET);
    info("Sending FACTORY_RESET to node %d (CAN ID 0x%03X)...\n", config.node_id, can_id);

    if (!can.send(can_id)) {
        fprintf(stderr, "Error: Failed to send FACTORY_RESET command\n");
        return 1;
    }

    info("FACTORY_RESET command sent successfully\n");
    return 0;
}

/* ============================================================================
 * Monitor Command - Message Parsing and Display
 * ========================================================================== */

// Parsed sensor data structures
struct AlsData {
    uint32_t lux;
    uint8_t status;
    uint8_t sequence;
    uint8_t config_idx;
    uint16_t checksum;
    bool checksum_valid;
};

struct EnvData {
    float temperature;  // °C
    uint8_t humidity;   // %RH
    float pressure;     // hPa
    uint8_t status;
    uint16_t checksum;
    bool checksum_valid;
};

struct AiqData {
    uint16_t iaq;
    uint8_t accuracy;
    uint16_t co2_equiv;
    uint16_t breath_voc;
    uint8_t status;
};

// Calculate checksum (same as ESP32 firmware)
static uint16_t calc_checksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

// Parse ALS message (0x0A2)
static bool parse_als_message(const struct can_frame& frame, AlsData& data) {
    if (frame.can_dlc < 8) return false;

    // Lux (uint24_t, little-endian)
    data.lux = frame.data[0] | (frame.data[1] << 8) | (frame.data[2] << 16);
    data.status = frame.data[3];
    data.sequence = frame.data[4];
    data.config_idx = frame.data[5];

    // Checksum (uint16_t, little-endian)
    data.checksum = frame.data[6] | (frame.data[7] << 8);
    uint16_t calc = calc_checksum(frame.data, 6);
    data.checksum_valid = (data.checksum == calc);

    return true;
}

// Parse Environmental message (0x0A3)
static bool parse_env_message(const struct can_frame& frame, EnvData& data) {
    if (frame.can_dlc < 8) return false;

    // Temperature (int16_t, °C × 100, little-endian)
    int16_t temp_raw = static_cast<int16_t>(frame.data[0] | (frame.data[1] << 8));
    data.temperature = temp_raw / 100.0f;

    // Humidity (uint8_t, %RH)
    data.humidity = frame.data[2];

    // Pressure (uint16_t, hPa × 10, little-endian)
    uint16_t press_raw = frame.data[3] | (frame.data[4] << 8);
    data.pressure = press_raw / 10.0f;

    data.status = frame.data[5];

    // Checksum (uint16_t, little-endian)
    data.checksum = frame.data[6] | (frame.data[7] << 8);
    uint16_t calc = calc_checksum(frame.data, 6);
    data.checksum_valid = (data.checksum == calc);

    return true;
}

// Parse Air Quality message (0x0A4)
static bool parse_aiq_message(const struct can_frame& frame, AiqData& data) {
    if (frame.can_dlc < 8) return false;

    // IAQ (uint16_t, little-endian)
    data.iaq = frame.data[0] | (frame.data[1] << 8);
    data.accuracy = frame.data[2];

    // CO2 equivalent (uint16_t, little-endian)
    data.co2_equiv = frame.data[3] | (frame.data[4] << 8);

    // Breath VOC (uint16_t, little-endian)
    data.breath_voc = frame.data[5] | (frame.data[6] << 8);

    data.status = frame.data[7];

    return true;
}

// Get sensor type name from config index
static const char* get_sensor_type(uint8_t config_idx) {
    if (config_idx <= 20) return "VEML7700";
    if (config_idx >= 100 && config_idx <= 111) return "OPT4001";
    return "Unknown";
}

// Get IAQ quality description
static const char* get_iaq_quality(uint16_t iaq) {
    if (iaq <= 50) return "Good";
    if (iaq <= 100) return "Average";
    if (iaq <= 150) return "Little Bad";
    if (iaq <= 200) return "Bad";
    if (iaq <= 300) return "Worse";
    return "Very Bad";
}

// Output formatters
static void output_als_human(const AlsData& data, int node_id) {
    printf("[Node %d] ALS (%s): %u lux, seq=%u, config=%u, status=%s%s\n",
           node_id,
           get_sensor_type(data.config_idx),
           data.lux,
           data.sequence,
           data.config_idx,
           data.status == 0 ? "OK" : "Error",
           data.checksum_valid ? "" : " [CHECKSUM ERROR]");
}

static void output_als_json(const AlsData& data, int node_id) {
    printf("{\"type\":\"als\",\"node\":%d,\"sensor\":\"%s\",\"lux\":%u,"
           "\"sequence\":%u,\"config\":%u,\"status\":%u,\"checksum_valid\":%s}\n",
           node_id,
           get_sensor_type(data.config_idx),
           data.lux,
           data.sequence,
           data.config_idx,
           data.status,
           data.checksum_valid ? "true" : "false");
}

static void output_als_csv(const AlsData& data, int node_id) {
    printf("als,%d,%s,%u,%u,%u,%u,%d\n",
           node_id,
           get_sensor_type(data.config_idx),
           data.lux,
           data.sequence,
           data.config_idx,
           data.status,
           data.checksum_valid ? 1 : 0);
}

static void output_env_human(const EnvData& data, int node_id) {
    printf("[Node %d] ENV: %.2f°C, %u%% RH, %.1f hPa, status=%s%s\n",
           node_id,
           data.temperature,
           data.humidity,
           data.pressure,
           data.status == 0 ? "OK" : "Error",
           data.checksum_valid ? "" : " [CHECKSUM ERROR]");
}

static void output_env_json(const EnvData& data, int node_id) {
    printf("{\"type\":\"env\",\"node\":%d,\"temperature\":%.2f,\"humidity\":%u,"
           "\"pressure\":%.1f,\"status\":%u,\"checksum_valid\":%s}\n",
           node_id,
           data.temperature,
           data.humidity,
           data.pressure,
           data.status,
           data.checksum_valid ? "true" : "false");
}

static void output_env_csv(const EnvData& data, int node_id) {
    printf("env,%d,%.2f,%u,%.1f,%u,%d\n",
           node_id,
           data.temperature,
           data.humidity,
           data.pressure,
           data.status,
           data.checksum_valid ? 1 : 0);
}

static void output_aiq_human(const AiqData& data, int node_id) {
    printf("[Node %d] AIQ: IAQ=%u (%s), accuracy=%u, CO2=%u ppm, VOC=%u ppm, status=%s\n",
           node_id,
           data.iaq,
           get_iaq_quality(data.iaq),
           data.accuracy,
           data.co2_equiv,
           data.breath_voc,
           data.status == 0 ? "OK" : "Error");
}

static void output_aiq_json(const AiqData& data, int node_id) {
    printf("{\"type\":\"aiq\",\"node\":%d,\"iaq\":%u,\"iaq_quality\":\"%s\","
           "\"accuracy\":%u,\"co2_ppm\":%u,\"voc_ppm\":%u,\"status\":%u}\n",
           node_id,
           data.iaq,
           get_iaq_quality(data.iaq),
           data.accuracy,
           data.co2_equiv,
           data.breath_voc,
           data.status);
}

static void output_aiq_csv(const AiqData& data, int node_id) {
    printf("aiq,%d,%u,%u,%u,%u,%u\n",
           node_id,
           data.iaq,
           data.accuracy,
           data.co2_equiv,
           data.breath_voc,
           data.status);
}

static void print_csv_header() {
    printf("# CSV format:\n");
    printf("# als,node,sensor,lux,sequence,config,status,checksum_valid\n");
    printf("# env,node,temperature,humidity,pressure,status,checksum_valid\n");
    printf("# aiq,node,iaq,accuracy,co2_ppm,voc_ppm,status\n");
}

// Signal handling for clean exit
#include <signal.h>
static volatile bool g_running = true;

static void signal_handler(int /* sig */) {
    g_running = false;
}

int cmd_monitor(CanSocket& can, const Config& config) {
    // Calculate CAN IDs for this node's sensor messages
    uint32_t als_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::ALS_DATA);
    uint32_t env_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::ENV_DATA);
    uint32_t aiq_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::AIR_QUALITY);

    info("Monitoring node %d (CAN IDs: ALS=0x%03X, ENV=0x%03X, AIQ=0x%03X)\n",
         config.node_id, als_id, env_id, aiq_id);

    if (config.count > 0) {
        info("Will stop after %d messages\n", config.count);
    }
    if (config.timeout > 0) {
        info("Will stop after %d seconds\n", config.timeout);
    }

    // Print CSV header if needed
    if (config.format == OutputFormat::CSV && !config.quiet) {
        print_csv_header();
    }

    // Setup signal handler for clean Ctrl+C
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Timing
    auto start_time = std::chrono::steady_clock::now();
    int msg_count = 0;

    // Main receive loop
    while (g_running) {
        // Check timeout
        if (config.timeout > 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            if (elapsed >= config.timeout) {
                info("\nTimeout reached (%d seconds)\n", config.timeout);
                break;
            }
        }

        // Receive with 100ms timeout (allows checking for Ctrl+C and timeout)
        struct can_frame frame;
        if (!can.receive(frame, 100)) {
            continue;  // Timeout or error, check loop conditions
        }

        // Check if message is from our target node
        int msg_node = can_protocol::extract_node_id(frame.can_id);
        if (msg_node != config.node_id) {
            continue;  // Message from different node, ignore
        }

        // Parse and output based on message type
        can_protocol::MsgOffset offset = can_protocol::extract_offset(frame.can_id);

        switch (offset) {
            case can_protocol::MsgOffset::ALS_DATA: {
                AlsData data;
                if (parse_als_message(frame, data)) {
                    switch (config.format) {
                        case OutputFormat::HUMAN: output_als_human(data, config.node_id); break;
                        case OutputFormat::JSON:  output_als_json(data, config.node_id); break;
                        case OutputFormat::CSV:   output_als_csv(data, config.node_id); break;
                    }
                    msg_count++;
                }
                break;
            }
            case can_protocol::MsgOffset::ENV_DATA: {
                EnvData data;
                if (parse_env_message(frame, data)) {
                    switch (config.format) {
                        case OutputFormat::HUMAN: output_env_human(data, config.node_id); break;
                        case OutputFormat::JSON:  output_env_json(data, config.node_id); break;
                        case OutputFormat::CSV:   output_env_csv(data, config.node_id); break;
                    }
                    msg_count++;
                }
                break;
            }
            case can_protocol::MsgOffset::AIR_QUALITY: {
                AiqData data;
                if (parse_aiq_message(frame, data)) {
                    switch (config.format) {
                        case OutputFormat::HUMAN: output_aiq_human(data, config.node_id); break;
                        case OutputFormat::JSON:  output_aiq_json(data, config.node_id); break;
                        case OutputFormat::CSV:   output_aiq_csv(data, config.node_id); break;
                    }
                    msg_count++;
                }
                break;
            }
            default:
                // Ignore other message types
                break;
        }

        // Flush output for real-time display
        fflush(stdout);

        // Check message count limit
        if (config.count > 0 && msg_count >= config.count) {
            info("\nMessage count reached (%d messages)\n", config.count);
            break;
        }
    }

    if (!g_running) {
        info("\nInterrupted\n");
    }

    info("Total messages received: %d\n", msg_count);
    return 0;
}

int cmd_set_node_id(CanSocket& /* can */, const Config& /* config */) {
    // TODO: Implement in Phase 4
    fprintf(stderr, "Error: set-node-id command not yet implemented\n");
    return 1;
}

int cmd_info(CanSocket& /* can */, const Config& /* config */) {
    // TODO: Implement in Phase 4
    fprintf(stderr, "Error: info command not yet implemented\n");
    return 1;
}

int cmd_discover(CanSocket& /* can */, const Config& /* config */) {
    // TODO: Implement in Phase 4
    fprintf(stderr, "Error: discover command not yet implemented\n");
    return 1;
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(int argc, char* argv[]) {
    Config config;

    // Parse arguments
    if (!parse_args(argc, argv, config)) {
        return 1;
    }

    // Set global quiet flag
    g_quiet = config.quiet;

    // Handle help and version (no CAN needed)
    if (config.command == Command::HELP) {
        print_usage(argv[0]);
        return 0;
    }
    if (config.command == Command::VERSION) {
        print_version();
        return 0;
    }

    // Open CAN socket
    CanSocket can;
    if (!can.open(config.interface)) {
        fprintf(stderr, "Error: Failed to open CAN interface '%s'\n", config.interface.c_str());
        fprintf(stderr, "Make sure the interface exists and is up:\n");
        fprintf(stderr, "  sudo ip link set %s type can bitrate %d\n", config.interface.c_str(), config.speed);
        fprintf(stderr, "  sudo ip link set %s up\n", config.interface.c_str());
        return 1;
    }

    info("Connected to %s (node %d, base address 0x%03X)\n",
         config.interface.c_str(), config.node_id,
         can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::STOP));

    // Execute command
    int result = 0;
    switch (config.command) {
        case Command::START:
            result = cmd_start(can, config);
            break;
        case Command::STOP:
            result = cmd_stop(can, config);
            break;
        case Command::SHUTDOWN:
            result = cmd_shutdown(can, config);
            break;
        case Command::REBOOT:
            result = cmd_reboot(can, config);
            break;
        case Command::FACTORY_RESET:
            result = cmd_factory_reset(can, config);
            break;
        case Command::MONITOR:
            result = cmd_monitor(can, config);
            break;
        case Command::SET_NODE_ID:
            result = cmd_set_node_id(can, config);
            break;
        case Command::INFO:
            result = cmd_info(can, config);
            break;
        case Command::DISCOVER:
            result = cmd_discover(can, config);
            break;
        default:
            fprintf(stderr, "Error: No command specified\n");
            result = 1;
            break;
    }

    return result;
}
