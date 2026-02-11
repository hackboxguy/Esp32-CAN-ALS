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

#include <fstream>

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

// Node addressing: each node has 32 message IDs (0x20 spacing)
constexpr uint32_t NODE_ADDR_SPACING = 0x20;
constexpr uint32_t BASE_ADDR_NODE_0 = 0x100;
constexpr int MAX_NODE_ID = 15;

// Message offsets from node base address
// Sensor data: 0x00-0x0F, Control commands: 0x10-0x1F
enum class MsgOffset : uint8_t {
    // Sensor data offsets (0x00-0x0F)
    ALS_DATA       = 0x00,  // Ambient light sensor data
    ENV_DATA       = 0x01,  // Environmental data (T/H/P)
    AIR_QUALITY    = 0x02,  // Air quality (IAQ/CO2/VOC)
    GAS_1          = 0x03,  // BME688 selectivity gas class 1
    GAS_2          = 0x04,  // BME688 selectivity gas class 2
    GAS_3          = 0x05,  // BME688 selectivity gas class 3
    GAS_4          = 0x06,  // BME688 selectivity gas class 4
    PRESENCE       = 0x07,  // mm-wave presence detection
    PRESENCE_EXT   = 0x08,  // Presence extended data
    // 0x09-0x0E reserved for future sensors
    STATUS         = 0x0F,  // System status

    // Control command offsets (0x10-0x1F)
    STOP           = 0x10,  // Stop transmission
    START          = 0x11,  // Start transmission
    SHUTDOWN       = 0x12,  // Graceful shutdown
    REBOOT         = 0x13,  // Reboot
    FACTORY_RESET  = 0x14,  // Factory reset
    SET_NODE_ID    = 0x15,  // Set node ID
    GET_INFO       = 0x16,  // Request device info
    INFO_RESPONSE  = 0x17,  // Device info response
    PING           = 0x18,  // Discovery ping
    PONG           = 0x19,  // Discovery response
    IDENTIFY       = 0x1A,  // Identify (blink LED)
    // 0x1B-0x1F reserved for future commands
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
 * OTA Protocol Constants
 * ========================================================================== */

namespace ota_protocol {

// OTA CAN ID addressing (separate range from sensor data)
constexpr uint32_t OTA_BASE_ADDR = 0x700;
constexpr uint32_t OTA_ADDR_SPACING = 0x10;
constexpr uint32_t OTA_RESP_OFFSET = 0x08;

// Calculate OTA CAN IDs for a given node
inline uint32_t cmd_id(int node_id) {
    return OTA_BASE_ADDR + (node_id * OTA_ADDR_SPACING);
}
inline uint32_t resp_id(int node_id) {
    return cmd_id(node_id) + OTA_RESP_OFFSET;
}

// OTA Command types (Tool -> ESP32)
constexpr uint8_t CMD_START_UPDATE  = 0x01;
constexpr uint8_t CMD_SEND_CHUNK    = 0x02;
constexpr uint8_t CMD_FINISH_UPDATE = 0x03;
constexpr uint8_t CMD_ACTIVATE_FW   = 0x04;
constexpr uint8_t CMD_GET_STATUS    = 0x05;
constexpr uint8_t CMD_ABORT_UPDATE  = 0x06;

// OTA Response types (ESP32 -> Tool)
constexpr uint8_t RESP_ACK      = 0x81;
constexpr uint8_t RESP_NAK      = 0x82;
constexpr uint8_t RESP_STATUS   = 0x83;
constexpr uint8_t RESP_READY    = 0x84;
constexpr uint8_t RESP_COMPLETE = 0x85;

// OTA Error codes
constexpr uint8_t ERR_OK        = 0x00;
constexpr uint8_t ERR_BUSY      = 0x01;
constexpr uint8_t ERR_NO_SPACE  = 0x02;
constexpr uint8_t ERR_SEQ       = 0x03;
constexpr uint8_t ERR_CRC       = 0x04;
constexpr uint8_t ERR_WRITE     = 0x05;
constexpr uint8_t ERR_INVALID   = 0x06;
constexpr uint8_t ERR_TIMEOUT   = 0x07;
constexpr uint8_t ERR_ABORTED   = 0x08;

// Activate flags
constexpr uint8_t ACTIVATE_FLAG_REBOOT = 0x01;

// Protocol constants
constexpr size_t CHUNK_SIZE = 6;          // Bytes per CAN frame payload
constexpr int ACK_TIMEOUT_MS = 150;       // Timeout for ACK response (reduced from 500ms)
constexpr int ACK_TIMEOUT_INITIAL_MS = 50;// Initial timeout for exponential backoff
constexpr int START_TIMEOUT_MS = 10000;   // Timeout for partition erase
constexpr int FINISH_TIMEOUT_MS = 5000;   // Timeout for CRC verification
constexpr int MAX_RETRIES = 3;            // Max retries per chunk
constexpr int CHUNK_DELAY_MS = 0;         // Delay between chunks (0 = fastest)

// Get error code name
inline const char* error_name(uint8_t err) {
    switch (err) {
        case ERR_OK:       return "OK";
        case ERR_BUSY:     return "BUSY";
        case ERR_NO_SPACE: return "NO_SPACE";
        case ERR_SEQ:      return "SEQUENCE_ERROR";
        case ERR_CRC:      return "CRC_ERROR";
        case ERR_WRITE:    return "WRITE_ERROR";
        case ERR_INVALID:  return "INVALID";
        case ERR_TIMEOUT:  return "TIMEOUT";
        case ERR_ABORTED:  return "ABORTED";
        default:           return "UNKNOWN";
    }
}

} // namespace ota_protocol

/* ============================================================================
 * CRC32 Calculation (for OTA)
 * ========================================================================== */

static uint32_t crc32_table[256];
static bool crc32_table_init = false;

static void init_crc32_table() {
    if (crc32_table_init) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
        }
        crc32_table[i] = crc;
    }
    crc32_table_init = true;
}

static uint32_t calculate_crc32(const uint8_t* data, size_t len, uint32_t crc = 0) {
    init_crc32_table();
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return ~crc;
}

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
    UPDATE,
    SANITY_TEST,
    IDENTIFY,
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

    // Update options
    std::string firmware_file;
    bool no_activate = false;
    bool no_reboot = false;
    bool verify_only = false;
    int chunk_delay_ms = ota_protocol::CHUNK_DELAY_MS;
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
    printf("    identify            Blink onboard LED for 5 seconds\n");
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
    printf("  Firmware Update:\n");
    printf("    update <file>       Upload firmware via CAN\n");
    printf("      --no-activate     Write firmware but don't activate\n");
    printf("      --no-reboot       Activate but don't reboot\n");
    printf("      --verify-only     Just verify file CRC, no upload\n");
    printf("      --chunk-delay=<ms> Delay between chunks (default: %d)\n", ota_protocol::CHUNK_DELAY_MS);
    printf("\n");
    printf("  Testing:\n");
    printf("    sanity-test <file>  Run comprehensive device tests\n");
    printf("                        (always targets node 0, continues on failure)\n");
    printf("\n");
    printf("EXAMPLES:\n");
    printf("  %s monitor                        # Monitor node 0\n", prog_name);
    printf("  %s --node-id=1 reboot             # Reboot node 1\n", prog_name);
    printf("  %s --interface=can1 discover      # Discover nodes on can1\n", prog_name);
    printf("  %s monitor --format=json --quiet  # JSON output for scripting\n", prog_name);
    printf("  %s update firmware.bin            # Update node 0 firmware\n", prog_name);
    printf("  %s --node-id=2 update fw.bin      # Update node 2 firmware\n", prog_name);
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

        // --no-activate (update option)
        if (strcmp(arg, "--no-activate") == 0) {
            config.no_activate = true;
            continue;
        }

        // --no-reboot (update option)
        if (strcmp(arg, "--no-reboot") == 0) {
            config.no_reboot = true;
            continue;
        }

        // --verify-only (update option)
        if (strcmp(arg, "--verify-only") == 0) {
            config.verify_only = true;
            continue;
        }

        // --chunk-delay=
        if (auto val = parse_option(arg, "--chunk-delay")) {
            auto delay = parse_int(val->c_str());
            if (!delay || *delay < 0) {
                fprintf(stderr, "Error: Invalid chunk-delay value: %s\n", val->c_str());
                return false;
            }
            config.chunk_delay_ms = *delay;
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
    } else if (cmd == "update") {
        config.command = Command::UPDATE;
        if (positional.size() < 2) {
            fprintf(stderr, "Error: update requires a firmware file argument\n");
            return false;
        }
        config.firmware_file = positional[1];
    } else if (cmd == "sanity-test") {
        config.command = Command::SANITY_TEST;
        if (positional.size() < 2) {
            fprintf(stderr, "Error: sanity-test requires a firmware file argument\n");
            return false;
        }
        config.firmware_file = positional[1];
    } else if (cmd == "identify") {
        config.command = Command::IDENTIFY;
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

int cmd_identify(CanSocket& can, const Config& config) {
    uint32_t can_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::IDENTIFY);
    info("Sending IDENTIFY to node %d (CAN ID 0x%03X)...\n", config.node_id, can_id);

    if (!can.send(can_id)) {
        fprintf(stderr, "Error: Failed to send IDENTIFY command\n");
        return 1;
    }

    info("IDENTIFY command sent - node %d LED will blink for 5 seconds\n", config.node_id);
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

// Parse ALS message (0x100)
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

// Parse Environmental message (0x101)
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

// Parse Air Quality message (0x102)
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
    if (config_idx >= 200 && config_idx <= 211) return "OPT3001";
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

/* ============================================================================
 * INFO Response Parsing
 * ========================================================================== */

struct InfoResponse {
    uint8_t node_id;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t fw_patch;
    uint8_t sensor_flags;
    uint8_t als_type;
    uint8_t status_flags;
    uint8_t partition_info;  // bits 0-2: type, bits 4-6: OTA state
};

// Partition info constants (must match ESP32 can_protocol.h)
constexpr uint8_t PARTITION_TYPE_FACTORY = 0;
constexpr uint8_t PARTITION_TYPE_OTA_0   = 1;
constexpr uint8_t PARTITION_TYPE_OTA_1   = 2;
constexpr uint8_t PARTITION_TYPE_UNKNOWN = 7;

constexpr uint8_t OTA_IMG_STATE_UNDEFINED = 0;
constexpr uint8_t OTA_IMG_STATE_NEW       = 1;
constexpr uint8_t OTA_IMG_STATE_PENDING   = 2;
constexpr uint8_t OTA_IMG_STATE_VALID     = 3;
constexpr uint8_t OTA_IMG_STATE_INVALID   = 4;
constexpr uint8_t OTA_IMG_STATE_ABORTED   = 5;

static bool parse_info_response(const struct can_frame& frame, InfoResponse& info) {
    if (frame.can_dlc < 7) return false;

    info.node_id = frame.data[0];
    info.fw_major = frame.data[1];
    info.fw_minor = frame.data[2];
    info.fw_patch = frame.data[3];
    info.sensor_flags = frame.data[4];
    info.als_type = frame.data[5];
    info.status_flags = frame.data[6];
    info.partition_info = (frame.can_dlc >= 8) ? frame.data[7] : 0;

    return true;
}

static const char* get_als_type_name(uint8_t als_type) {
    switch (als_type) {
        case 0: return "None";
        case 1: return "VEML7700";
        case 2: return "OPT4001";
        case 3: return "OPT3001";
        default: return "Unknown";
    }
}

static const char* get_partition_type_name(uint8_t partition_info) {
    uint8_t type = partition_info & 0x07;
    switch (type) {
        case PARTITION_TYPE_FACTORY: return "factory";
        case PARTITION_TYPE_OTA_0:   return "ota_0";
        case PARTITION_TYPE_OTA_1:   return "ota_1";
        default:                     return "unknown";
    }
}

static const char* get_ota_state_name(uint8_t partition_info) {
    uint8_t state = (partition_info >> 4) & 0x07;
    switch (state) {
        case OTA_IMG_STATE_UNDEFINED: return "undefined";
        case OTA_IMG_STATE_NEW:       return "new";
        case OTA_IMG_STATE_PENDING:   return "pending";
        case OTA_IMG_STATE_VALID:     return "valid";
        case OTA_IMG_STATE_INVALID:   return "invalid";
        case OTA_IMG_STATE_ABORTED:   return "aborted";
        default:                      return "unknown";
    }
}

static void print_info_response(const InfoResponse& info, OutputFormat format) {
    bool has_als = (info.sensor_flags & 0x01) != 0;
    bool has_bme = (info.sensor_flags & 0x02) != 0;
    bool is_bme688 = (info.sensor_flags & 0x10) != 0;  // Bit 4 indicates BME688 variant
    bool has_ld2410 = (info.sensor_flags & 0x04) != 0;
    bool has_mq3 = (info.sensor_flags & 0x08) != 0;
    bool tx_active = (info.status_flags & 0x01) != 0;
    const char* bme_name = is_bme688 ? "BME688" : "BME680";

    const char* partition = get_partition_type_name(info.partition_info);
    const char* ota_state = get_ota_state_name(info.partition_info);

    switch (format) {
        case OutputFormat::HUMAN:
            printf("Device Info (Node %d):\n", info.node_id);
            printf("  Firmware:    v%d.%d.%d\n", info.fw_major, info.fw_minor, info.fw_patch);
            printf("  Partition:   %s (%s)\n", partition, ota_state);
            printf("  Base Addr:   0x%03X\n", can_protocol::BASE_ADDR_NODE_0 + info.node_id * can_protocol::NODE_ADDR_SPACING);
            printf("  Sensors:     ");
            if (has_als) printf("ALS(%s) ", get_als_type_name(info.als_type));
            if (has_bme) printf("%s ", bme_name);
            if (has_ld2410) printf("LD2410 ");
            if (has_mq3) printf("MQ-3 ");
            if (!has_als && !has_bme && !has_ld2410 && !has_mq3) printf("(none)");
            printf("\n");
            printf("  TX Active:   %s\n", tx_active ? "Yes" : "No");
            break;

        case OutputFormat::JSON:
            printf("{\"type\":\"info\",\"node\":%d,\"firmware\":\"%d.%d.%d\","
                   "\"partition\":\"%s\",\"ota_state\":\"%s\","
                   "\"base_addr\":\"0x%03X\",\"sensors\":{\"als\":%s,\"als_type\":\"%s\","
                   "\"bme\":%s,\"bme_type\":\"%s\",\"ld2410\":%s,\"mq3\":%s},\"tx_active\":%s}\n",
                   info.node_id, info.fw_major, info.fw_minor, info.fw_patch,
                   partition, ota_state,
                   can_protocol::BASE_ADDR_NODE_0 + info.node_id * can_protocol::NODE_ADDR_SPACING,
                   has_als ? "true" : "false", get_als_type_name(info.als_type),
                   has_bme ? "true" : "false", has_bme ? bme_name : "None",
                   has_ld2410 ? "true" : "false",
                   has_mq3 ? "true" : "false",
                   tx_active ? "true" : "false");
            break;

        case OutputFormat::CSV:
            printf("info,%d,%d.%d.%d,%s,%s,0x%03X,%d,%s,%d,%d,%d,%d\n",
                   info.node_id, info.fw_major, info.fw_minor, info.fw_patch,
                   partition, ota_state,
                   can_protocol::BASE_ADDR_NODE_0 + info.node_id * can_protocol::NODE_ADDR_SPACING,
                   has_als ? 1 : 0, get_als_type_name(info.als_type),
                   has_bme ? 1 : 0, has_ld2410 ? 1 : 0, has_mq3 ? 1 : 0,
                   tx_active ? 1 : 0);
            break;
    }
}

/* ============================================================================
 * Set Node ID Command
 * ========================================================================== */

int cmd_set_node_id(CanSocket& can, const Config& config) {
    uint32_t can_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::SET_NODE_ID);

    info("Setting node %d to new ID %d (CAN ID 0x%03X)...\n",
         config.node_id, config.new_node_id, can_id);

    uint8_t data[8] = {0};
    data[0] = static_cast<uint8_t>(config.new_node_id);

    if (!can.send(can_id, data, 8)) {
        fprintf(stderr, "Error: Failed to send SET_NODE_ID command\n");
        return 1;
    }

    info("SET_NODE_ID command sent. Device will reboot with new node ID %d.\n",
         config.new_node_id);
    info("New base address will be 0x%03X\n",
         can_protocol::BASE_ADDR_NODE_0 + config.new_node_id * can_protocol::NODE_ADDR_SPACING);

    return 0;
}

/* ============================================================================
 * Info Command
 * ========================================================================== */

int cmd_info(CanSocket& can, const Config& config) {
    uint32_t get_info_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::GET_INFO);
    uint32_t info_resp_id = can_protocol::make_can_id(config.node_id, can_protocol::MsgOffset::INFO_RESPONSE);

    info("Querying node %d info (CAN ID 0x%03X)...\n", config.node_id, get_info_id);

    // Send GET_INFO request
    if (!can.send(get_info_id)) {
        fprintf(stderr, "Error: Failed to send GET_INFO command\n");
        return 1;
    }

    // Wait for INFO_RESPONSE (500ms timeout)
    struct can_frame frame;
    auto start = std::chrono::steady_clock::now();
    const int timeout_ms = 500;

    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();

        if (elapsed >= timeout_ms) {
            fprintf(stderr, "Error: No response from node %d (timeout)\n", config.node_id);
            return 1;
        }

        int remaining = timeout_ms - static_cast<int>(elapsed);
        if (can.receive(frame, remaining)) {
            if (frame.can_id == info_resp_id) {
                InfoResponse resp;
                if (parse_info_response(frame, resp)) {
                    print_info_response(resp, config.format);
                    return 0;
                }
            }
        }
    }
}

/* ============================================================================
 * Discover Command
 * ========================================================================== */

int cmd_discover(CanSocket& can, const Config& config) {
    info("Discovering sensor nodes on %s...\n", can.interface().c_str());

    // Track which nodes responded
    bool node_found[can_protocol::MAX_NODE_ID + 1] = {false};
    int nodes_found = 0;

    // Send PING to all possible node addresses
    for (int node = 0; node <= can_protocol::MAX_NODE_ID; node++) {
        uint32_t ping_id = can_protocol::make_can_id(node, can_protocol::MsgOffset::PING);
        can.send(ping_id);
    }

    // Wait for PONG responses (500ms total, enough for 16 nodes)
    auto start = std::chrono::steady_clock::now();
    const int timeout_ms = 500;

    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();

        if (elapsed >= timeout_ms) {
            break;
        }

        struct can_frame frame;
        int remaining = timeout_ms - static_cast<int>(elapsed);
        if (can.receive(frame, remaining)) {
            // Check if this is a PONG response
            int node = can_protocol::extract_node_id(frame.can_id);
            if (node >= 0 && node <= can_protocol::MAX_NODE_ID) {
                can_protocol::MsgOffset offset = can_protocol::extract_offset(frame.can_id);
                if (offset == can_protocol::MsgOffset::PONG && !node_found[node]) {
                    node_found[node] = true;
                    nodes_found++;

                    uint8_t reported_node = frame.data[0];

                    switch (config.format) {
                        case OutputFormat::HUMAN:
                            printf("  Node %d found (base 0x%03X)\n", reported_node,
                                   can_protocol::BASE_ADDR_NODE_0 + reported_node * can_protocol::NODE_ADDR_SPACING);
                            break;
                        case OutputFormat::JSON:
                            printf("{\"type\":\"discover\",\"node\":%d,\"base_addr\":\"0x%03X\"}\n",
                                   reported_node,
                                   can_protocol::BASE_ADDR_NODE_0 + reported_node * can_protocol::NODE_ADDR_SPACING);
                            break;
                        case OutputFormat::CSV:
                            printf("discover,%d,0x%03X\n", reported_node,
                                   can_protocol::BASE_ADDR_NODE_0 + reported_node * can_protocol::NODE_ADDR_SPACING);
                            break;
                    }
                }
            }
        }
    }

    if (config.format == OutputFormat::HUMAN) {
        if (nodes_found == 0) {
            printf("No sensor nodes found.\n");
        } else {
            printf("\nFound %d node(s).\n", nodes_found);
        }
    }

    return 0;
}

/* ============================================================================
 * Update Command - OTA Firmware Update
 * ========================================================================== */

// Progress bar helper
static void print_progress(uint32_t current, uint32_t total, uint32_t bytes_per_sec) {
    const int bar_width = 30;
    float progress = static_cast<float>(current) / total;
    int filled = static_cast<int>(progress * bar_width);

    fprintf(stderr, "\rUploading: [");
    for (int i = 0; i < bar_width; i++) {
        if (i < filled) fprintf(stderr, "=");
        else if (i == filled) fprintf(stderr, ">");
        else fprintf(stderr, " ");
    }
    fprintf(stderr, "] %3d%% (%u/%u) %.1f KB/s  ",
            static_cast<int>(progress * 100), current, total,
            bytes_per_sec / 1024.0f);
    fflush(stderr);
}

int cmd_update(CanSocket& can, const Config& config) {
    using namespace ota_protocol;

    // Read firmware file
    std::ifstream file(config.firmware_file, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        fprintf(stderr, "Error: Cannot open firmware file: %s\n", config.firmware_file.c_str());
        return 1;
    }

    std::streamsize fw_size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> firmware(fw_size);
    if (!file.read(reinterpret_cast<char*>(firmware.data()), fw_size)) {
        fprintf(stderr, "Error: Failed to read firmware file\n");
        return 1;
    }
    file.close();

    info("Reading firmware file: %s (%u bytes)\n", config.firmware_file.c_str(), static_cast<uint32_t>(fw_size));

    // Calculate CRC32
    uint32_t crc32 = calculate_crc32(firmware.data(), firmware.size());
    uint16_t crc16 = static_cast<uint16_t>(crc32 & 0xFFFF);  // Truncated for START_UPDATE

    info("Firmware CRC32: 0x%08X\n", crc32);

    // Verify-only mode
    if (config.verify_only) {
        printf("Firmware verification complete.\n");
        printf("  File: %s\n", config.firmware_file.c_str());
        printf("  Size: %u bytes\n", static_cast<uint32_t>(fw_size));
        printf("  CRC32: 0x%08X\n", crc32);
        return 0;
    }

    // CAN IDs for this node
    uint32_t cmd_can_id = cmd_id(config.node_id);
    uint32_t resp_can_id = resp_id(config.node_id);

    info("Connecting to node %d (OTA CAN ID: 0x%03X)...\n", config.node_id, cmd_can_id);

    // Send START_UPDATE: [Type:1][Seq:1][Size:4][CRC16:2]
    {
        uint8_t data[8] = {0};
        data[0] = CMD_START_UPDATE;
        data[1] = 0;  // Sequence (unused for START)
        data[2] = (fw_size >> 0) & 0xFF;
        data[3] = (fw_size >> 8) & 0xFF;
        data[4] = (fw_size >> 16) & 0xFF;
        data[5] = (fw_size >> 24) & 0xFF;
        data[6] = crc16 & 0xFF;
        data[7] = (crc16 >> 8) & 0xFF;

        if (!can.send(cmd_can_id, data, 8)) {
            fprintf(stderr, "Error: Failed to send START_UPDATE\n");
            return 1;
        }
    }

    info("Starting update (erasing partition)...\n");

    // Wait for READY response
    struct can_frame frame;
    auto start_time = std::chrono::steady_clock::now();
    bool got_ready = false;

    while (!got_ready) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed >= START_TIMEOUT_MS) {
            fprintf(stderr, "Error: Timeout waiting for READY response\n");
            return 1;
        }

        int remaining = START_TIMEOUT_MS - static_cast<int>(elapsed);
        if (can.receive(frame, remaining)) {
            if (frame.can_id == resp_can_id && frame.can_dlc >= 2) {
                uint8_t resp_type = frame.data[0];

                if (resp_type == RESP_READY) {
                    uint16_t max_chunk = frame.data[2] | (frame.data[3] << 8);
                    uint32_t free_space = frame.data[4] | (frame.data[5] << 8) |
                                         (frame.data[6] << 16) | (frame.data[7] << 24);
                    info("Partition ready (max_chunk=%u, free_space=%u)\n", max_chunk, free_space);
                    got_ready = true;
                } else if (resp_type == RESP_NAK) {
                    uint8_t err = frame.data[3];
                    fprintf(stderr, "Error: Device rejected START_UPDATE: %s\n", error_name(err));
                    return 1;
                }
            }
        }
    }

    // Send firmware chunks
    info("Uploading firmware...\n");

    uint32_t bytes_sent = 0;
    uint8_t seq = 0;
    int retries = 0;
    auto upload_start = std::chrono::steady_clock::now();

    while (bytes_sent < firmware.size()) {
        // Calculate chunk size
        size_t remaining_bytes = firmware.size() - bytes_sent;
        size_t chunk_len = (remaining_bytes < CHUNK_SIZE) ? remaining_bytes : CHUNK_SIZE;

        // Build SEND_CHUNK message
        uint8_t data[8] = {0};
        data[0] = CMD_SEND_CHUNK;
        data[1] = seq;
        std::memcpy(&data[2], &firmware[bytes_sent], chunk_len);

        // Send chunk
        if (!can.send(cmd_can_id, data, 8)) {
            fprintf(stderr, "\nError: Failed to send chunk %d\n", seq);
            return 1;
        }

        // Wait for ACK with exponential backoff on retries
        // First attempt: 50ms, retry 1: 100ms, retry 2: 150ms, retry 3: 150ms (capped)
        bool got_ack = false;
        int current_timeout = ACK_TIMEOUT_INITIAL_MS * (1 << retries);  // 50, 100, 200, 400...
        if (current_timeout > ACK_TIMEOUT_MS) current_timeout = ACK_TIMEOUT_MS;
        auto chunk_start = std::chrono::steady_clock::now();

        while (!got_ack) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - chunk_start).count();

            if (elapsed >= current_timeout) {
                retries++;
                if (retries > MAX_RETRIES) {
                    fprintf(stderr, "\nError: Max retries exceeded for chunk %d\n", seq);
                    return 1;
                }
                // Retry with increased timeout (exponential backoff)
                current_timeout = ACK_TIMEOUT_INITIAL_MS * (1 << retries);
                if (current_timeout > ACK_TIMEOUT_MS) current_timeout = ACK_TIMEOUT_MS;

                if (!can.send(cmd_can_id, data, 8)) {
                    fprintf(stderr, "\nError: Failed to resend chunk %d\n", seq);
                    return 1;
                }
                chunk_start = std::chrono::steady_clock::now();
                continue;
            }

            int wait_ms = current_timeout - static_cast<int>(elapsed);
            if (can.receive(frame, wait_ms)) {
                if (frame.can_id == resp_can_id && frame.can_dlc >= 2) {
                    uint8_t resp_type = frame.data[0];
                    uint8_t resp_seq = frame.data[2];

                    if (resp_type == RESP_ACK && resp_seq == seq) {
                        got_ack = true;
                        bytes_sent += chunk_len;
                        seq = (seq + 1) & 0xFF;
                        retries = 0;

                        // Update progress
                        auto now = std::chrono::steady_clock::now();
                        auto elapsed_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - upload_start).count() / 1000.0f;
                        uint32_t bytes_per_sec = (elapsed_sec > 0) ?
                            static_cast<uint32_t>(bytes_sent / elapsed_sec) : 0;
                        print_progress(bytes_sent, static_cast<uint32_t>(firmware.size()), bytes_per_sec);

                    } else if (resp_type == RESP_NAK) {
                        uint8_t err = frame.data[3];
                        if (err == ERR_SEQ) {
                            // Sequence mismatch - try to resync
                            retries++;
                            if (retries > MAX_RETRIES) {
                                fprintf(stderr, "\nError: Sequence error, max retries exceeded\n");
                                return 1;
                            }
                        } else {
                            fprintf(stderr, "\nError: Device NAK'd chunk %d: %s\n", seq, error_name(err));
                            return 1;
                        }
                    }
                }
            }
        }

        // Inter-chunk delay
        if (config.chunk_delay_ms > 0 && bytes_sent < firmware.size()) {
            usleep(config.chunk_delay_ms * 1000);
        }
    }

    fprintf(stderr, "\n");  // End progress line
    info("Upload complete, verifying CRC...\n");

    // Send FINISH_UPDATE with full CRC32
    {
        uint8_t data[8] = {0};
        data[0] = CMD_FINISH_UPDATE;
        data[1] = 0;  // Sequence (unused)
        data[2] = (crc32 >> 0) & 0xFF;
        data[3] = (crc32 >> 8) & 0xFF;
        data[4] = (crc32 >> 16) & 0xFF;
        data[5] = (crc32 >> 24) & 0xFF;

        if (!can.send(cmd_can_id, data, 8)) {
            fprintf(stderr, "Error: Failed to send FINISH_UPDATE\n");
            return 1;
        }
    }

    // Wait for COMPLETE response
    start_time = std::chrono::steady_clock::now();
    bool got_complete = false;
    uint8_t result = 0xFF;

    while (!got_complete) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed >= FINISH_TIMEOUT_MS) {
            fprintf(stderr, "Error: Timeout waiting for COMPLETE response\n");
            return 1;
        }

        int remaining = FINISH_TIMEOUT_MS - static_cast<int>(elapsed);
        if (can.receive(frame, remaining)) {
            if (frame.can_id == resp_can_id && frame.can_dlc >= 2) {
                uint8_t resp_type = frame.data[0];

                if (resp_type == RESP_COMPLETE) {
                    result = frame.data[2];
                    got_complete = true;
                } else if (resp_type == RESP_NAK) {
                    uint8_t err = frame.data[3];
                    fprintf(stderr, "Error: FINISH_UPDATE failed: %s\n", error_name(err));
                    return 1;
                }
            }
        }
    }

    if (result != ERR_OK) {
        fprintf(stderr, "Error: Firmware verification failed: %s\n", error_name(result));
        return 1;
    }

    info("Firmware verified successfully.\n");

    // Skip activation if requested
    if (config.no_activate) {
        info("Firmware written but not activated (--no-activate).\n");
        info("Use 'can-sensor-tool --node-id=%d update --activate' to activate later.\n", config.node_id);
        return 0;
    }

    // Send ACTIVATE_FW
    info("Activating new firmware...\n");
    {
        uint8_t flags = config.no_reboot ? 0 : ACTIVATE_FLAG_REBOOT;
        uint8_t data[8] = {0};
        data[0] = CMD_ACTIVATE_FW;
        data[1] = 0;  // Sequence (unused)
        data[2] = flags;

        if (!can.send(cmd_can_id, data, 8)) {
            fprintf(stderr, "Error: Failed to send ACTIVATE_FW\n");
            return 1;
        }
    }

    // Wait for ACK
    start_time = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed >= ACK_TIMEOUT_MS) {
            // Node may have rebooted already, which is OK
            break;
        }

        int remaining = ACK_TIMEOUT_MS - static_cast<int>(elapsed);
        if (can.receive(frame, remaining)) {
            if (frame.can_id == resp_can_id && frame.data[0] == RESP_ACK) {
                break;
            }
        }
    }

    if (config.no_reboot) {
        info("Firmware activated. Use 'can-sensor-tool --node-id=%d reboot' to boot new firmware.\n",
             config.node_id);
    } else {
        info("Update complete! Node %d is rebooting to new firmware.\n", config.node_id);
    }

    return 0;
}

/* ============================================================================
 * Sanity Test Command
 * ========================================================================== */

struct TestResult {
    std::string name;
    bool passed;
    std::string message;
    int duration_ms;
};

/* ---- Sanity Test Helper Functions ---- */

static bool verify_node_responds(CanSocket& can, int node_id, int timeout_ms) {
    uint32_t ping_id = can_protocol::make_can_id(node_id, can_protocol::MsgOffset::PING);
    uint32_t pong_id = can_protocol::make_can_id(node_id, can_protocol::MsgOffset::PONG);

    if (!can.send(ping_id)) return false;

    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_ms) return false;

        struct can_frame frame;
        int remaining = timeout_ms - static_cast<int>(elapsed);
        if (can.receive(frame, remaining)) {
            if (frame.can_id == pong_id) return true;
        }
    }
}

static bool wait_for_node_ready(CanSocket& can, int node_id, int timeout_ms) {
    uint32_t ping_id = can_protocol::make_can_id(node_id, can_protocol::MsgOffset::PING);
    uint32_t pong_id = can_protocol::make_can_id(node_id, can_protocol::MsgOffset::PONG);

    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_ms) return false;

        can.send(ping_id);

        struct can_frame frame;
        if (can.receive(frame, 200)) {
            if (frame.can_id == pong_id) return true;
        }
        usleep(100000);  // 100ms between retries
    }
}

static bool get_node_info(CanSocket& can, int node_id, InfoResponse& resp, int timeout_ms) {
    uint32_t get_info_id = can_protocol::make_can_id(node_id, can_protocol::MsgOffset::GET_INFO);
    uint32_t info_resp_id = can_protocol::make_can_id(node_id, can_protocol::MsgOffset::INFO_RESPONSE);

    if (!can.send(get_info_id)) return false;

    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_ms) return false;

        struct can_frame frame;
        int remaining = timeout_ms - static_cast<int>(elapsed);
        if (can.receive(frame, remaining)) {
            if (frame.can_id == info_resp_id) {
                return parse_info_response(frame, resp);
            }
        }
    }
}

static int count_als_messages(CanSocket& can, int node_id, int duration_ms) {
    uint32_t als_id = can_protocol::make_can_id(node_id, can_protocol::MsgOffset::ALS_DATA);
    auto start = std::chrono::steady_clock::now();
    int count = 0;

    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        int remaining = duration_ms - static_cast<int>(elapsed);
        if (remaining <= 0) break;

        struct can_frame frame;
        if (can.receive(frame, remaining)) {
            if (frame.can_id == als_id) count++;
        }
    }
    return count;
}

static int measure_als_interval(CanSocket& can, int node_id, int sample_duration_ms) {
    uint32_t als_id = can_protocol::make_can_id(node_id, can_protocol::MsgOffset::ALS_DATA);
    std::vector<std::chrono::steady_clock::time_point> timestamps;
    auto start = std::chrono::steady_clock::now();

    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        int remaining = sample_duration_ms - static_cast<int>(elapsed);
        if (remaining <= 0) break;

        struct can_frame frame;
        if (can.receive(frame, remaining)) {
            if (frame.can_id == als_id) {
                timestamps.push_back(std::chrono::steady_clock::now());
            }
        }
    }

    if (timestamps.size() < 3) return -1;

    int total_interval = 0;
    for (size_t i = 1; i < timestamps.size(); i++) {
        total_interval += static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            timestamps[i] - timestamps[i - 1]).count());
    }
    return total_interval / static_cast<int>(timestamps.size() - 1);
}

// Drain any pending CAN frames (call between tests to avoid stale data)
static void drain_can_frames(CanSocket& can, int drain_ms = 50) {
    struct can_frame frame;
    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= drain_ms) break;
        can.receive(frame, 10);
    }
}

/* ---- Individual Test Functions ---- */

static TestResult test_discovery(CanSocket& can, int target_node) {
    auto start = std::chrono::steady_clock::now();
    bool ok = verify_node_responds(can, target_node, 500);
    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (ok) return {"Discovery", true, "Node 0 responds to PING", dur};
    return {"Discovery", false, "Node 0 did not respond to PING", dur};
}

static TestResult test_device_info(CanSocket& can, int target_node) {
    auto start = std::chrono::steady_clock::now();
    InfoResponse resp;
    bool ok = get_node_info(can, target_node, resp, 500);
    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (ok) {
        char msg[256];
        snprintf(msg, sizeof(msg), "FW v%d.%d.%d, sensors=0x%02X, ALS=%s, partition=%s (%s)",
                 resp.fw_major, resp.fw_minor, resp.fw_patch,
                 resp.sensor_flags, get_als_type_name(resp.als_type),
                 get_partition_type_name(resp.partition_info),
                 get_ota_state_name(resp.partition_info));
        return {"Device Info", true, msg, dur};
    }
    return {"Device Info", false, "No INFO_RESPONSE received", dur};
}

static TestResult test_start_stop(CanSocket& can, int target_node) {
    auto start = std::chrono::steady_clock::now();

    // Send STOP first to ensure clean state
    uint32_t stop_id = can_protocol::make_can_id(target_node, can_protocol::MsgOffset::STOP);
    can.send(stop_id);
    drain_can_frames(can, 200);

    // Send START
    uint32_t start_id = can_protocol::make_can_id(target_node, can_protocol::MsgOffset::START);
    if (!can.send(start_id)) {
        return {"Start/Stop", false, "Failed to send START", 0};
    }
    usleep(100000);  // 100ms settle

    // Count ALS messages over 3 seconds
    int msg_count = count_als_messages(can, target_node, 3000);

    // Send STOP
    can.send(stop_id);
    usleep(200000);  // 200ms settle
    drain_can_frames(can, 100);

    // Verify silence for 1.5 seconds
    int stopped_count = count_als_messages(can, target_node, 1500);

    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (msg_count >= 2 && msg_count <= 5 && stopped_count == 0) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Received %d ALS msgs in 3s, 0 after STOP", msg_count);
        return {"Start/Stop", true, msg, dur};
    } else if (msg_count < 2) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Too few ALS messages: %d (expected 2-5)", msg_count);
        return {"Start/Stop", false, msg, dur};
    } else if (stopped_count > 0) {
        char msg[128];
        snprintf(msg, sizeof(msg), "%d messages received after STOP", stopped_count);
        return {"Start/Stop", false, msg, dur};
    }
    char msg[128];
    snprintf(msg, sizeof(msg), "Unexpected count: %d msgs, %d after stop", msg_count, stopped_count);
    return {"Start/Stop", false, msg, dur};
}

static TestResult test_monitor_interval(CanSocket& can, int target_node) {
    auto start = std::chrono::steady_clock::now();

    // Send START
    uint32_t start_id = can_protocol::make_can_id(target_node, can_protocol::MsgOffset::START);
    can.send(start_id);
    usleep(100000);

    // Measure interval over 5 seconds
    int avg_interval = measure_als_interval(can, target_node, 5000);

    // Send STOP
    uint32_t stop_id = can_protocol::make_can_id(target_node, can_protocol::MsgOffset::STOP);
    can.send(stop_id);
    drain_can_frames(can, 200);

    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (avg_interval < 0) {
        return {"Monitor Interval", false, "Too few messages to measure interval", dur};
    }
    if (avg_interval >= 800 && avg_interval <= 1200) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Average interval: %d ms (target: 1000 ms)", avg_interval);
        return {"Monitor Interval", true, msg, dur};
    }
    char msg[128];
    snprintf(msg, sizeof(msg), "Interval out of range: %d ms (expected 800-1200)", avg_interval);
    return {"Monitor Interval", false, msg, dur};
}

static TestResult test_reboot(CanSocket& can, int target_node) {
    auto start = std::chrono::steady_clock::now();

    uint32_t reboot_id = can_protocol::make_can_id(target_node, can_protocol::MsgOffset::REBOOT);
    if (!can.send(reboot_id)) {
        return {"Reboot", false, "Failed to send REBOOT", 0};
    }

    // Wait a moment for reboot to start, then poll
    usleep(500000);
    bool recovered = wait_for_node_ready(can, target_node, 5000);

    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (recovered) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Node rebooted and recovered in %d ms", dur);
        return {"Reboot", true, msg, dur};
    }
    return {"Reboot", false, "Node did not respond after reboot (5s timeout)", dur};
}

static TestResult test_set_node_id(CanSocket& can, int target_node) {
    auto start = std::chrono::steady_clock::now();

    // Step 1: Change node 0 -> node 1
    uint32_t set_id_cmd = can_protocol::make_can_id(0, can_protocol::MsgOffset::SET_NODE_ID);
    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    if (!can.send(set_id_cmd, data, 8)) {
        return {"Set Node ID", false, "Failed to send SET_NODE_ID (0->1)", 0};
    }

    // Wait for reboot
    usleep(500000);
    if (!wait_for_node_ready(can, 1, 5000)) {
        // Try to recover: node might still be at 0
        return {"Set Node ID", false, "Node did not respond on ID 1 after change", 0};
    }

    // Verify node 1 reports correct ID
    InfoResponse info1;
    if (!get_node_info(can, 1, info1, 500) || info1.node_id != 1) {
        // Try to change back anyway
        uint32_t set_id_1 = can_protocol::make_can_id(1, can_protocol::MsgOffset::SET_NODE_ID);
        data[0] = 0;
        can.send(set_id_1, data, 8);
        usleep(3000000);
        return {"Set Node ID", false, "Node on ID 1 reported wrong node_id", 0};
    }

    // Step 2: Change node 1 -> node 0
    uint32_t set_id_1 = can_protocol::make_can_id(1, can_protocol::MsgOffset::SET_NODE_ID);
    data[0] = 0;
    if (!can.send(set_id_1, data, 8)) {
        return {"Set Node ID", false, "Failed to send SET_NODE_ID (1->0)", 0};
    }

    // Wait for reboot
    usleep(500000);
    if (!wait_for_node_ready(can, 0, 5000)) {
        return {"Set Node ID", false, "Node did not respond on ID 0 after restore", 0};
    }

    // Verify node 0 reports correct ID
    InfoResponse info0;
    if (!get_node_info(can, 0, info0, 500) || info0.node_id != 0) {
        return {"Set Node ID", false, "Node on ID 0 reported wrong node_id after restore", 0};
    }

    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());
    return {"Set Node ID", true, "Successfully changed 0->1->0", dur};
}

static TestResult test_factory_reset(CanSocket& can, int target_node) {
    auto start = std::chrono::steady_clock::now();

    uint32_t reset_id = can_protocol::make_can_id(target_node, can_protocol::MsgOffset::FACTORY_RESET);
    if (!can.send(reset_id)) {
        return {"Factory Reset", false, "Failed to send FACTORY_RESET", 0};
    }

    usleep(500000);
    bool recovered = wait_for_node_ready(can, target_node, 5000);

    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (recovered) {
        char msg[128];
        snprintf(msg, sizeof(msg), "Factory reset completed in %d ms", dur);
        return {"Factory Reset", true, msg, dur};
    }
    return {"Factory Reset", false, "Node did not respond after factory reset", dur};
}

static TestResult test_ota_update(CanSocket& can, int target_node, const std::string& firmware_file) {
    auto start = std::chrono::steady_clock::now();

    // Build a config for cmd_update
    Config temp_config;
    temp_config.interface = "can0";  // Not used (socket already open)
    temp_config.node_id = target_node;
    temp_config.firmware_file = firmware_file;
    temp_config.no_activate = false;
    temp_config.no_reboot = false;
    temp_config.verify_only = false;
    temp_config.chunk_delay_ms = ota_protocol::CHUNK_DELAY_MS;
    temp_config.quiet = false;

    int result = cmd_update(can, temp_config);

    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (result != 0) {
        return {"OTA Update", false, "OTA update failed", dur};
    }

    // Wait for node to boot with new firmware
    usleep(2000000);  // 2 seconds
    InfoResponse resp;
    bool ok = get_node_info(can, target_node, resp, 2000);

    dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (ok) {
        float speed_kbps = 0;
        // Estimate speed from file size and duration
        std::ifstream f(firmware_file, std::ios::binary | std::ios::ate);
        if (f.is_open()) {
            size_t fsize = f.tellg();
            f.close();
            speed_kbps = (fsize / 1024.0f) / (dur / 1000.0f);
        }
        char msg[256];
        snprintf(msg, sizeof(msg), "OTA OK, node booted v%d.%d.%d on %s (%.1f KB/s, %d ms)",
                 resp.fw_major, resp.fw_minor, resp.fw_patch,
                 get_partition_type_name(resp.partition_info),
                 speed_kbps, dur);
        return {"OTA Update", true, msg, dur};
    }
    return {"OTA Update", false, "Update succeeded but node did not respond after boot", dur};
}

static TestResult test_final_discovery(CanSocket& can, int target_node) {
    auto start = std::chrono::steady_clock::now();
    bool ok = verify_node_responds(can, target_node, 500);
    int dur = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count());

    if (ok) return {"Final Discovery", true, "Node 0 still responds after all tests", dur};
    return {"Final Discovery", false, "Node 0 unresponsive after tests", dur};
}

/* ---- Sanity Test Orchestrator ---- */

int cmd_sanity_test(CanSocket& can, const Config& config) {
    const int TARGET_NODE = 0;
    const int TOTAL_TESTS = 9;

    fprintf(stderr, "========================================\n");
    fprintf(stderr, "  CAN SENSOR NODE SANITY TEST\n");
    fprintf(stderr, "========================================\n");
    fprintf(stderr, "Target node: %d (base 0x%03X)\n", TARGET_NODE,
            can_protocol::BASE_ADDR_NODE_0);
    fprintf(stderr, "Firmware:    %s\n", config.firmware_file.c_str());
    fprintf(stderr, "========================================\n\n");

    std::vector<TestResult> results;
    auto overall_start = std::chrono::steady_clock::now();

    // Run all tests
    auto run_test = [&](int num, const char* label, TestResult (*fn)(CanSocket&, int)) {
        fprintf(stderr, "[%d/%d] %s...\n", num, TOTAL_TESTS, label);
        results.push_back(fn(can, TARGET_NODE));
        const auto& r = results.back();
        fprintf(stderr, "       %s: %s\n\n",
                r.passed ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m", r.message.c_str());
        drain_can_frames(can);
    };

    run_test(1, "Discovery", test_discovery);
    run_test(2, "Device Info", test_device_info);
    run_test(3, "Start/Stop", test_start_stop);
    run_test(4, "Monitor Interval", test_monitor_interval);
    run_test(5, "Reboot", test_reboot);
    run_test(6, "Set Node ID", test_set_node_id);
    run_test(7, "Factory Reset", test_factory_reset);

    // OTA test needs firmware file - different signature
    fprintf(stderr, "[8/%d] OTA Update...\n", TOTAL_TESTS);
    results.push_back(test_ota_update(can, TARGET_NODE, config.firmware_file));
    {
        const auto& r = results.back();
        fprintf(stderr, "       %s: %s\n\n",
                r.passed ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m", r.message.c_str());
    }
    drain_can_frames(can);

    run_test(9, "Final Discovery", test_final_discovery);

    // Calculate summary
    auto overall_end = std::chrono::steady_clock::now();
    int total_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        overall_end - overall_start).count());
    int passed = 0, failed = 0;
    for (const auto& r : results) {
        if (r.passed) passed++; else failed++;
    }

    // Print summary
    fprintf(stderr, "========================================\n");
    fprintf(stderr, "  TEST SUMMARY\n");
    fprintf(stderr, "========================================\n");
    for (size_t i = 0; i < results.size(); i++) {
        const auto& r = results[i];
        fprintf(stderr, "  [%zu] %-20s %s  (%d ms)\n",
                i + 1, r.name.c_str(),
                r.passed ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m", r.duration_ms);
        if (!r.passed) {
            fprintf(stderr, "      -> %s\n", r.message.c_str());
        }
    }
    fprintf(stderr, "========================================\n");
    fprintf(stderr, "  Results: %d/%d passed, %d failed\n", passed, TOTAL_TESTS, failed);
    fprintf(stderr, "  Total time: %.1f seconds\n", total_ms / 1000.0f);
    fprintf(stderr, "========================================\n");

    // Cleanup: ensure node ID is back to 0
    InfoResponse cleanup_info;
    if (get_node_info(can, 0, cleanup_info, 500) && cleanup_info.node_id == 0) {
        // Already at node 0, nothing to do
    } else {
        fprintf(stderr, "\n  Cleanup: Restoring node ID to 0...\n");
        // Try sending SET_NODE_ID=0 to nodes 0-15
        for (int n = 0; n < 16; n++) {
            uint32_t cmd = can_protocol::make_can_id(n, can_protocol::MsgOffset::SET_NODE_ID);
            uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            can.send(cmd, data, 8);
        }
        usleep(500000);
        if (wait_for_node_ready(can, 0, 5000)) {
            fprintf(stderr, "  Cleanup: Node ID restored to 0\n");
        } else {
            fprintf(stderr, "  \033[31mCleanup: WARNING - Failed to restore node ID to 0!\033[0m\n");
        }
    }

    return (failed > 0) ? 1 : 0;
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
         can_protocol::BASE_ADDR_NODE_0 + config.node_id * can_protocol::NODE_ADDR_SPACING);

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
        case Command::UPDATE:
            result = cmd_update(can, config);
            break;
        case Command::SANITY_TEST:
            result = cmd_sanity_test(can, config);
            break;
        case Command::IDENTIFY:
            result = cmd_identify(can, config);
            break;
        default:
            fprintf(stderr, "Error: No command specified\n");
            result = 1;
            break;
    }

    return result;
}
