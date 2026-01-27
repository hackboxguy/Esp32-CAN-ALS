# Factory/Main Firmware Restructure Plan

This document describes the restructuring of the ESP32-CAN-Sensor firmware to support two distinct build configurations:

1. **FACTORY firmware** - Minimal bootloader with OTA support only
2. **MAIN firmware** - Full-featured sensor firmware

## Goals

- **FACTORY**: Minimal size (<128KB), CAN communication + OTA updates only
- **MAIN**: Full sensor support (~295KB), all features enabled
- Single codebase with conditional compilation
- Single partition table for both firmware types
- Separate build directories to avoid constant rebuilds

## Partition Table Layout

### New Layout (4MB Flash)

| Address | Size | Partition | Notes |
|---------|------|-----------|-------|
| 0x009000 | 24KB (0x6000) | nvs | Non-volatile storage |
| 0x00F000 | 4KB (0x1000) | phy_init | PHY calibration data |
| 0x010000 | 8KB (0x2000) | otadata | OTA state tracking |
| 0x020000 | 128KB (0x20000) | factory | Minimal FACTORY firmware |
| 0x040000 | 1.75MB (0x1C0000) | ota_0 | MAIN firmware slot A |
| 0x200000 | 1.75MB (0x1C0000) | ota_1 | MAIN firmware slot B |

**Total used**: 3.75MB (0x3C0000)
**Spare**: 256KB (headroom for future expansion)

### Previous Layout (for reference)

| Partition | Old Size | New Size | Change |
|-----------|----------|----------|--------|
| factory | 1MB | 128KB | -896KB |
| ota_0 | 1MB | 1.75MB | +768KB |
| ota_1 | 1MB | 1.75MB | +768KB |

## Firmware Feature Comparison

| Feature | FACTORY | MAIN |
|---------|---------|------|
| NVS flash management | ✅ | ✅ |
| Node ID load/save | ✅ | ✅ |
| CAN (TWAI) driver | ✅ | ✅ |
| OTA firmware updates | ✅ | ✅ |
| Device info (GET_INFO) | ✅ | ✅ |
| Discovery (PING/PONG) | ✅ | ✅ |
| Reboot command | ✅ | ✅ |
| Factory reset command | ✅ | ✅ |
| Set node ID command | ✅ | ✅ |
| ALS sensors (VEML/OPT) | ❌ | ✅ |
| BME680/688 + BSEC | ❌ | ✅ |
| Sensor data transmission | ❌ | ✅ |
| Auto-start on boot | ❌ | ✅ |
| Estimated size | ~100-120KB | ~295KB |

## CAN Bus Behavior

### FACTORY Firmware

- **Silent by default** - Does not auto-start transmission
- **Responds only when queried** by can-sensor-tool or direct CAN commands
- **Supported commands**:
  - `PING (0x0AE)` → Responds with PONG
  - `GET_INFO (0x0AC)` → Responds with device info (sensor_flags=0)
  - `OTA commands (0x700+)` → Full OTA update support
  - `REBOOT (0x0A9)` → Reboots ESP32
  - `FACTORY_RESET (0x0AA)` → Clears NVS, reboots
  - `SET_NODE_ID (0x0AB)` → Saves new node ID, reboots
- **Ignored commands** (no action, no response):
  - `START (0x0A1)` - No sensors to start
  - `STOP (0x0A0)` - Nothing running
  - `SHUTDOWN (0x0A8)` - No state to save

### MAIN Firmware

- **Auto-starts transmission** on boot (existing behavior)
- **Full sensor data** on CAN bus:
  - `0x0A2` - Ambient light (1 Hz)
  - `0x0A3` - Environmental T/H/P (0.33 Hz)
  - `0x0A4` - Air quality IAQ/CO2/VOC (0.33 Hz)
- **All commands supported** including START/STOP/SHUTDOWN

### Device Info Response Differences

| Field | FACTORY | MAIN |
|-------|---------|------|
| sensor_flags (byte 4) | 0x00 | Actual sensors detected |
| als_type (byte 5) | 0x00 (none) | 1/2/3 based on sensor |
| partition_info (byte 7) | 0=factory | 1 or 2 (ota_0/ota_1) |

## File Structure Changes

### New Files

```
main/
├── firmware_config.h          # NEW - Firmware type definitions
```

### Modified Files

```
main/
├── CMakeLists.txt             # Conditional source inclusion
├── main.c                     # Conditional sensor code
├── can_protocol.c             # Minor adjustments for factory
├── can_protocol.h             # Add FIRMWARE_TYPE_* defines
partitions.csv                 # New partition sizes
sdkconfig.defaults             # Renamed to sdkconfig.defaults.main
sdkconfig.defaults.factory     # NEW - Factory-specific config
```

### Files Conditionally Compiled (MAIN only)

```
main/
├── als_driver.c/h             # MAIN only
├── veml7700_driver.c/h        # MAIN only
├── opt4001_driver.c/h         # MAIN only
├── opt3001_driver.c/h         # MAIN only
├── bme680_bsec.c/h            # MAIN only
```

## Implementation Details

### 1. firmware_config.h

```c
#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

/* Firmware type values */
#define FIRMWARE_TYPE_FACTORY   0
#define FIRMWARE_TYPE_MAIN      1

/* Default to MAIN if not specified via CMAKE */
#ifndef FIRMWARE_TYPE
#define FIRMWARE_TYPE   FIRMWARE_TYPE_MAIN
#endif

/* Convenience macros for conditional compilation */
#define IS_FACTORY_FIRMWARE (FIRMWARE_TYPE == FIRMWARE_TYPE_FACTORY)
#define IS_MAIN_FIRMWARE    (FIRMWARE_TYPE == FIRMWARE_TYPE_MAIN)

/* Feature flags derived from firmware type */
#if IS_MAIN_FIRMWARE
#define SENSORS_ENABLED     1
#define AUTO_START_TX       1
#else
#define SENSORS_ENABLED     0
#define AUTO_START_TX       0
#endif

#endif /* FIRMWARE_CONFIG_H */
```

### 2. CMakeLists.txt Changes

```cmake
# main/CMakeLists.txt

# Check firmware type from command line
# Usage: idf.py build -DFIRMWARE_TYPE=FACTORY
if(NOT DEFINED FIRMWARE_TYPE)
    set(FIRMWARE_TYPE "MAIN")
endif()

message(STATUS "Building firmware type: ${FIRMWARE_TYPE}")

# Base sources (always included)
set(SRCS
    "main.c"
    "can_protocol.c"
    "ota_handler.c"
)

# Sensor sources (MAIN only)
if(FIRMWARE_TYPE STREQUAL "MAIN")
    list(APPEND SRCS
        "als_driver.c"
        "veml7700_driver.c"
        "opt4001_driver.c"
        "opt3001_driver.c"
        "bme680_bsec.c"
    )
    set(REQUIRES driver esp_timer nvs_flash app_update esp_partition bsec)
else()
    # FACTORY - minimal dependencies
    set(REQUIRES driver esp_timer nvs_flash app_update esp_partition)
endif()

# Add firmware type as compile definition
if(FIRMWARE_TYPE STREQUAL "FACTORY")
    add_compile_definitions(FIRMWARE_TYPE=0)
else()
    add_compile_definitions(FIRMWARE_TYPE=1)
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)
```

### 3. main.c Conditional Compilation

Key sections to wrap with `#if IS_MAIN_FIRMWARE`:

```c
#include "firmware_config.h"

#if IS_MAIN_FIRMWARE
#include "als_driver.h"
#include "sensor_common.h"
#if defined(CONFIG_BME680_ENABLED)
#include "bme680_bsec.h"
#endif
#endif

/* Global state - conditional */
#if IS_MAIN_FIRMWARE
static QueueHandle_t sensor_queue;
static bool g_tx_active = false;
#endif

/* Sensor poll task - MAIN only */
#if IS_MAIN_FIRMWARE
static void sensor_poll_task(void *arg) {
    // ... existing implementation
}
#endif

/* BME680 task - MAIN only */
#if IS_MAIN_FIRMWARE && defined(CONFIG_BME680_ENABLED)
static void bme680_sensor_task(void *arg) {
    // ... existing implementation
}
#endif

/* Transmit task - simplified for FACTORY */
static void twai_transmit_task(void *arg) {
#if IS_MAIN_FIRMWARE
    // Full implementation with sensor data
#else
    // FACTORY: Task not needed, or minimal heartbeat
    vTaskDelete(NULL);
#endif
}

/* Control task - conditional auto-start */
static void twai_control_task(void *arg) {
#if AUTO_START_TX
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreGive(start_sem);
    ESP_LOGI(TAG, "Auto-start: Enabled sensor transmission");
#else
    ESP_LOGI(TAG, "FACTORY mode: Waiting for commands (no auto-start)");
#endif
    vTaskDelete(NULL);
}

void app_main(void) {
    // Core init (both firmware types)
    nvs_flash_init();
    load_node_id_from_nvs(&g_node_id);
    ota_handler_init(g_node_id);

    // TWAI init (both)
    twai_driver_install(...);
    twai_start();

#if IS_MAIN_FIRMWARE
    // Sensor init (MAIN only)
    sensor_queue = xQueueCreate(SENSOR_QUEUE_DEPTH, sizeof(sensor_data_t));
    als_init();
    #if defined(CONFIG_BME680_ENABLED)
    bme680_init();
    #endif

    // Sensor tasks (MAIN only)
    xTaskCreate(sensor_poll_task, "SENSOR_POLL", 3072, NULL, SENSOR_POLL_TASK_PRIO, NULL);
    #if defined(CONFIG_BME680_ENABLED)
    xTaskCreate(bme680_sensor_task, "BME680_POLL", 10240, NULL, BME680_TASK_PRIO, NULL);
    #endif
#endif

    // CAN tasks (both, but transmit task is minimal for FACTORY)
    xTaskCreate(twai_receive_task, "TWAI_RX", 3072, NULL, CAN_RX_TASK_PRIO, NULL);
#if IS_MAIN_FIRMWARE
    xTaskCreate(twai_transmit_task, "TWAI_TX", 5120, NULL, CAN_TX_TASK_PRIO, NULL);
#endif
    xTaskCreate(twai_control_task, "TWAI_CTRL", 2048, NULL, CAN_CTRL_TSK_PRIO, NULL);
}
```

### 4. Receive Task Changes for FACTORY

```c
static void twai_receive_task(void *arg) {
    while (1) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            uint32_t id = rx_msg.identifier;

#if IS_MAIN_FIRMWARE
            // Full command handling including START/STOP
            if (id == my_stop_id) {
                // ... handle stop
            } else if (id == my_start_id) {
                // ... handle start
            } else if (id == my_shutdown_id) {
                // ... handle shutdown with BSEC save
#else
            // FACTORY: Ignore START/STOP/SHUTDOWN silently
            if (id == my_stop_id || id == my_start_id || id == my_shutdown_id) {
                ESP_LOGD(TAG, "FACTORY mode: Ignoring command 0x%03lX", id);
#endif
            } else if (id == my_reboot_id) {
                // Both handle reboot
            } else if (id == my_factory_id) {
                // Both handle factory reset (FACTORY doesn't have BSEC to clear)
            } else if (id == my_setid_id) {
                // Both handle set node ID
            } else if (id == my_getinfo_id) {
                // Both respond to GET_INFO
                // FACTORY: sensor_flags=0, als_type=0
            } else if (id == my_ping_id) {
                // Both respond to PING
            } else {
                // OTA handling (both)
                ota_handler_process_message(&rx_msg);
            }
        }
    }
}
```

### 5. Helper Functions for FACTORY

```c
static uint8_t get_sensor_flags(void) {
#if IS_MAIN_FIRMWARE
    uint8_t flags = 0;
    if (als_get_sensor_name() != NULL) {
        flags |= SENSOR_FLAG_ALS;
    }
    #if defined(CONFIG_BME680_ENABLED)
    flags |= SENSOR_FLAG_BME680;
    if (bme680_is_bme688()) {
        flags |= SENSOR_FLAG_BME688;
    }
    #endif
    return flags;
#else
    return 0;  /* FACTORY: No sensors */
#endif
}

static uint8_t get_als_type(void) {
#if IS_MAIN_FIRMWARE
    const char *name = als_get_sensor_name();
    if (name == NULL) return ALS_TYPE_NONE;
    if (strstr(name, "VEML7700")) return ALS_TYPE_VEML7700;
    if (strstr(name, "OPT4001")) return ALS_TYPE_OPT4001;
    if (strstr(name, "OPT3001")) return ALS_TYPE_OPT3001;
#endif
    return ALS_TYPE_NONE;
}
```

### 6. sdkconfig Files

#### sdkconfig.defaults.main

```ini
# sdkconfig.defaults.main - Full-featured MAIN firmware

# Partition table
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y

# Enable all sensors
CONFIG_BME680_ENABLED=y

# Performance optimization
CONFIG_COMPILER_OPTIMIZATION_PERF=y

# Logging
CONFIG_LOG_DEFAULT_LEVEL_INFO=y

# FreeRTOS
CONFIG_FREERTOS_HZ=100
```

#### sdkconfig.defaults.factory

```ini
# sdkconfig.defaults.factory - Minimal FACTORY firmware

# Partition table (same as main)
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y

# DISABLE sensors
CONFIG_BME680_ENABLED=n

# Size optimization (critical for 128KB limit)
CONFIG_COMPILER_OPTIMIZATION_SIZE=y

# Reduce logging
CONFIG_LOG_DEFAULT_LEVEL_WARN=y
CONFIG_BOOTLOADER_LOG_LEVEL_WARN=y

# Disable unused features
CONFIG_ESP_WIFI_ENABLED=n
CONFIG_BT_ENABLED=n

# Reduce stack sizes
CONFIG_ESP_MAIN_TASK_STACK_SIZE=3072
CONFIG_FREERTOS_IDLE_TASK_STACKSIZE=1024

# Disable unused components
CONFIG_VFS_SUPPORT_IO=n
CONFIG_VFS_SUPPORT_DIR=n
CONFIG_VFS_SUPPORT_SELECT=n

# Minimal newlib
CONFIG_NEWLIB_NANO_FORMAT=y
```

### 7. Updated partitions.csv

```csv
# ESP32 Partition Table for Factory + A/B OTA Updates
# Name,   Type, SubType,  Offset,    Size,      Flags
nvs,      data, nvs,      0x9000,    0x6000,
phy_init, data, phy,      0xf000,    0x1000,
otadata,  data, ota,      0x10000,   0x2000,
factory,  app,  factory,  0x20000,   0x20000,
ota_0,    app,  ota_0,    0x40000,   0x1C0000,
ota_1,    app,  ota_1,    0x200000,  0x1C0000,
```

## Build Commands

### Build FACTORY Firmware

```bash
# Clean factory build (first time or after switching)
idf.py -B build_factory fullclean

# Configure with factory defaults
cp sdkconfig.defaults.factory sdkconfig.defaults
idf.py -B build_factory set-target esp32c6

# Build factory firmware
idf.py -B build_factory build \
    -DFIRMWARE_TYPE=FACTORY \
    -DFIRMWARE_VERSION_MAJOR=1 \
    -DFIRMWARE_VERSION_MINOR=0 \
    -DFIRMWARE_VERSION_PATCH=0

# Flash factory firmware (first time setup)
idf.py -B build_factory -p /dev/ttyUSB0 flash

# Rename output binary
cp build_factory/esp32-can-als.bin build_factory/esp32-can-als-factory.bin
```

### Build MAIN Firmware

```bash
# Clean main build (first time or after switching)
idf.py -B build_main fullclean

# Configure with main defaults
cp sdkconfig.defaults.main sdkconfig.defaults
idf.py -B build_main set-target esp32c6

# Build main firmware
idf.py -B build_main build \
    -DFIRMWARE_TYPE=MAIN \
    -DFIRMWARE_VERSION_MAJOR=2 \
    -DFIRMWARE_VERSION_MINOR=0 \
    -DFIRMWARE_VERSION_PATCH=1

# OTA update via can-sensor-tool (normal operation)
can-sensor-tool --node=0 --ota=build_main/esp32-can-als.bin

# Or rename for clarity
cp build_main/esp32-can-als.bin build_main/esp32-can-als-main.bin
```

### Quick Build Script (build.sh)

```bash
#!/bin/bash
# Usage: ./build.sh [factory|main] [version]
# Example: ./build.sh factory 1.0.0
# Example: ./build.sh main 2.0.1

TYPE=${1:-main}
VERSION=${2:-1.0.0}

IFS='.' read -r MAJOR MINOR PATCH <<< "$VERSION"

if [ "$TYPE" = "factory" ]; then
    BUILD_DIR="build_factory"
    SDKCONFIG="sdkconfig.defaults.factory"
    FIRMWARE_TYPE="FACTORY"
else
    BUILD_DIR="build_main"
    SDKCONFIG="sdkconfig.defaults.main"
    FIRMWARE_TYPE="MAIN"
fi

echo "Building $TYPE firmware v$VERSION..."
cp $SDKCONFIG sdkconfig.defaults

idf.py -B $BUILD_DIR build \
    -DFIRMWARE_TYPE=$FIRMWARE_TYPE \
    -DFIRMWARE_VERSION_MAJOR=$MAJOR \
    -DFIRMWARE_VERSION_MINOR=$MINOR \
    -DFIRMWARE_VERSION_PATCH=$PATCH

cp $BUILD_DIR/esp32-can-als.bin $BUILD_DIR/esp32-can-als-${TYPE}.bin
echo "Output: $BUILD_DIR/esp32-can-als-${TYPE}.bin"
```

## Deployment Workflow

### Initial Device Setup

1. **Flash FACTORY firmware** via USB/Serial (one-time):
   ```bash
   idf.py -B build_factory -p /dev/ttyUSB0 flash
   ```

2. **Verify FACTORY is running**:
   ```bash
   can-sensor-tool --ping
   can-sensor-tool --node=0 --info
   # Should show: sensor_flags=0, partition=factory
   ```

3. **OTA update to MAIN firmware**:
   ```bash
   can-sensor-tool --node=0 --ota=build_main/esp32-can-als-main.bin
   ```

4. **Verify MAIN is running**:
   ```bash
   can-sensor-tool --node=0 --info
   # Should show: sensor_flags with detected sensors, partition=ota_0
   ```

### Normal OTA Update Cycle

```bash
# Build new MAIN firmware
./build.sh main 2.1.0

# Update device over CAN
can-sensor-tool --node=0 --ota=build_main/esp32-can-als-main.bin

# Verify update
can-sensor-tool --node=0 --info
```

### Recovery (if MAIN firmware fails)

If a MAIN firmware update fails validation or crashes:
1. ESP32 automatically rolls back to previous working firmware
2. If both OTA slots fail, device boots into FACTORY partition
3. FACTORY firmware allows new OTA update attempt

## Testing Checklist

### FACTORY Firmware Tests

- [ ] Binary size < 128KB
- [ ] Boots successfully
- [ ] Responds to PING
- [ ] Responds to GET_INFO (sensor_flags=0)
- [ ] OTA update works (receive MAIN firmware)
- [ ] REBOOT command works
- [ ] FACTORY_RESET command works
- [ ] SET_NODE_ID command works
- [ ] Silent on CAN bus (no auto-transmission)
- [ ] START/STOP commands ignored gracefully

### MAIN Firmware Tests

- [ ] Binary size < 1.75MB (should be ~295KB)
- [ ] OTA update from FACTORY works
- [ ] OTA update from MAIN (A→B, B→A) works
- [ ] All sensors detected and working
- [ ] Auto-starts transmission on boot
- [ ] All CAN commands functional
- [ ] Rollback works if validation fails

### Partition Table Tests

- [ ] FACTORY firmware fits in 128KB partition
- [ ] MAIN firmware fits in 1.75MB partition
- [ ] NVS works correctly
- [ ] OTA state tracking works
- [ ] A/B switching works correctly

## Size Budget

### FACTORY Firmware Target: < 128KB

| Component | Estimated Size |
|-----------|----------------|
| Bootloader overhead | ~20KB |
| FreeRTOS kernel | ~25KB |
| ESP-IDF core (nvs, timer) | ~40KB |
| TWAI driver | ~8KB |
| OTA handler | ~15KB |
| Our code (main + can_protocol) | ~8KB |
| **Total** | **~116KB** |
| **Headroom** | **~12KB** |

### MAIN Firmware Target: < 1.75MB

| Component | Estimated Size |
|-----------|----------------|
| Everything in FACTORY | ~116KB |
| BSEC library | ~80KB |
| Sensor drivers | ~20KB |
| Additional features | ~80KB |
| **Total** | **~296KB** |
| **Headroom** | **~1.5MB** |

## Version Numbering Convention

| Firmware | Version Range | Example |
|----------|---------------|---------|
| FACTORY | 1.x.x | 1.0.0, 1.0.1, 1.1.0 |
| MAIN | 2.x.x+ | 2.0.0, 2.1.0, 3.0.0 |

This allows easy identification of firmware type from version number.

## Rollback Scenarios

### Scenario 1: MAIN Update Fails Validation
- ESP-IDF detects invalid image
- Automatically boots previous MAIN (other OTA slot)
- If both OTA slots invalid → boots FACTORY

### Scenario 2: MAIN Crashes on Boot
- Watchdog triggers reboot
- After N failed boots, rolls back to previous
- User can force rollback via FACTORY_RESET from FACTORY

### Scenario 3: Intentional Rollback
- User sends FACTORY_RESET command
- Clears NVS, reboots
- Boots into FACTORY (if OTA slots cleared) or last valid MAIN

## Secure Boot Compatibility

### Overview

This partition layout is **fully compatible** with ESP32-C3/C6 Secure Boot v2 and Flash Encryption.

### Space Impact Analysis

| Security Feature | Space Overhead | Location |
|------------------|----------------|----------|
| App signature | ~4KB per image | Appended to binary |
| Bootloader signature | ~4KB | Bootloader partition |
| Flash encryption | 0 | Transparent (no partition change) |
| Secure boot keys | 0 | Stored in eFuse |

### Factory Firmware with Secure Boot

With signature overhead:
- Base firmware: ~116KB
- Signature block: ~4KB
- **Total: ~120KB** (fits in 128KB partition)

### Enabling Secure Boot (Future)

When ready to enable secure boot, add to sdkconfig:

```ini
# Secure Boot v2
CONFIG_SECURE_BOOT=y
CONFIG_SECURE_BOOT_V2_ENABLED=y
CONFIG_SECURE_SIGNED_ON_UPDATE=y
CONFIG_SECURE_SIGNED_ON_BOOT=y

# Flash Encryption (optional, for full protection)
CONFIG_SECURE_FLASH_ENC_ENABLED=y
CONFIG_SECURE_FLASH_ENCRYPTION_MODE_DEVELOPMENT=y
```

### Partition Headroom

Current layout provides adequate headroom:
- **Factory**: 128KB partition, ~120KB used with signing = 8KB spare
- **OTA slots**: 1.75MB each, plenty of room for signatures
- **Spare flash**: 256KB available for future security partitions

**Note**: If secure boot testing shows factory is too tight, we can increase
factory to 192KB by reducing OTA slots slightly (still >1.6MB each).

---

## can-sensor-tool Updates

### New --info Output Format

The tool will display firmware type based on partition info:

```
$ can-sensor-tool --node=0 --info

Device Info (Node 0):
  Firmware:    v1.0.0 (FACTORY)      ← NEW: Shows FACTORY/MAIN
  Partition:   factory
  OTA State:   n/a
  Sensors:     None                   ← FACTORY shows "None"
  ALS Type:    None
  Status:      Idle (not transmitting)
```

For MAIN firmware:

```
$ can-sensor-tool --node=0 --info

Device Info (Node 0):
  Firmware:    v2.0.1 (MAIN)         ← Shows MAIN
  Partition:   ota_0 (App-A)
  OTA State:   Valid
  Sensors:     ALS, BME680
  ALS Type:    VEML7700
  Status:      Transmitting
```

### Implementation Changes

In `can-sensor-tool.cpp`, update `cmd_info()`:

```cpp
// Determine firmware type from version or partition
const char* fw_type;
if (partition_type == PARTITION_TYPE_FACTORY) {
    fw_type = "FACTORY";
} else {
    fw_type = "MAIN";
}

printf("  Firmware:    v%d.%d.%d (%s)\n",
       ver_major, ver_minor, ver_patch, fw_type);

// Sensors display
if (sensor_flags == 0) {
    printf("  Sensors:     None\n");
} else {
    // ... existing sensor display code
}
```

---

## Build Script (build.sh)

### Full-Featured Build Script

```bash
#!/bin/bash
#
# build.sh - ESP32 CAN Sensor Firmware Build Script
#
# Usage:
#   ./build.sh --type=factory --version=1.0.0
#   ./build.sh --type=main --version=2.0.1 --target=esp32c3
#   ./build.sh --type=main --flash --port=/dev/ttyUSB0
#   ./build.sh --clean --type=factory
#
# Options:
#   --type=TYPE        Firmware type: factory or main (default: main)
#   --version=X.Y.Z    Version number (default: 1.0.0)
#   --target=TARGET    ESP32 target: esp32c3, esp32c6 (default: esp32c6)
#   --idfpath=PATH     Path to ESP-IDF (default: $IDF_PATH or ~/esp/esp-idf)
#   --port=PORT        Serial port for flashing (default: /dev/ttyUSB0)
#   --flash            Flash firmware after building
#   --monitor          Open serial monitor after flashing
#   --clean            Clean build directory before building
#   --fullclean        Full clean (reconfigure from scratch)
#   --size             Show size analysis after build
#   --help             Show this help message
#

set -e

# Default values
TYPE="main"
VERSION="1.0.0"
TARGET="esp32c6"
IDF_PATH_ARG=""
PORT="/dev/ttyUSB0"
DO_FLASH=false
DO_MONITOR=false
DO_CLEAN=false
DO_FULLCLEAN=false
DO_SIZE=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_usage() {
    head -30 "$0" | tail -28 | sed 's/^# //' | sed 's/^#//'
}

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --type=*)
            TYPE="${1#*=}"
            shift
            ;;
        --version=*)
            VERSION="${1#*=}"
            shift
            ;;
        --target=*)
            TARGET="${1#*=}"
            shift
            ;;
        --idfpath=*)
            IDF_PATH_ARG="${1#*=}"
            shift
            ;;
        --port=*)
            PORT="${1#*=}"
            shift
            ;;
        --flash)
            DO_FLASH=true
            shift
            ;;
        --monitor)
            DO_MONITOR=true
            shift
            ;;
        --clean)
            DO_CLEAN=true
            shift
            ;;
        --fullclean)
            DO_FULLCLEAN=true
            shift
            ;;
        --size)
            DO_SIZE=true
            shift
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Validate firmware type
if [[ "$TYPE" != "factory" && "$TYPE" != "main" ]]; then
    log_error "Invalid firmware type: $TYPE (must be 'factory' or 'main')"
    exit 1
fi

# Parse version
IFS='.' read -r MAJOR MINOR PATCH <<< "$VERSION"
if [[ -z "$MAJOR" || -z "$MINOR" || -z "$PATCH" ]]; then
    log_error "Invalid version format: $VERSION (must be X.Y.Z)"
    exit 1
fi

# Setup ESP-IDF environment
if [[ -n "$IDF_PATH_ARG" ]]; then
    export IDF_PATH="$IDF_PATH_ARG"
elif [[ -z "$IDF_PATH" ]]; then
    if [[ -d "$HOME/esp/esp-idf" ]]; then
        export IDF_PATH="$HOME/esp/esp-idf"
    else
        log_error "ESP-IDF not found. Set IDF_PATH or use --idfpath="
        exit 1
    fi
fi

# Source ESP-IDF export script if idf.py not available
if ! command -v idf.py &> /dev/null; then
    log_info "Sourcing ESP-IDF environment from $IDF_PATH"
    source "$IDF_PATH/export.sh"
fi

# Set build variables based on type
if [[ "$TYPE" == "factory" ]]; then
    BUILD_DIR="build_factory"
    SDKCONFIG_SRC="sdkconfig.defaults.factory"
    FIRMWARE_TYPE="FACTORY"
    OUTPUT_NAME="esp32-can-als-factory.bin"
else
    BUILD_DIR="build_main"
    SDKCONFIG_SRC="sdkconfig.defaults.main"
    FIRMWARE_TYPE="MAIN"
    OUTPUT_NAME="esp32-can-als-main.bin"
fi

# Print build configuration
echo ""
echo "=========================================="
echo "  ESP32 CAN Sensor Build Configuration"
echo "=========================================="
echo "  Type:      $TYPE ($FIRMWARE_TYPE)"
echo "  Version:   $VERSION"
echo "  Target:    $TARGET"
echo "  Build Dir: $BUILD_DIR"
echo "  IDF Path:  $IDF_PATH"
if $DO_FLASH; then
    echo "  Flash:     Yes (port: $PORT)"
fi
echo "=========================================="
echo ""

# Check if sdkconfig source exists
if [[ ! -f "$SDKCONFIG_SRC" ]]; then
    log_error "SDK config file not found: $SDKCONFIG_SRC"
    exit 1
fi

# Full clean if requested
if $DO_FULLCLEAN; then
    log_info "Performing full clean..."
    rm -rf "$BUILD_DIR"
    rm -f sdkconfig
fi

# Clean if requested
if $DO_CLEAN && [[ -d "$BUILD_DIR" ]]; then
    log_info "Cleaning build directory..."
    idf.py -B "$BUILD_DIR" fullclean
fi

# Copy appropriate sdkconfig
log_info "Configuring with $SDKCONFIG_SRC..."
cp "$SDKCONFIG_SRC" sdkconfig.defaults

# Set target if build directory doesn't exist or after fullclean
if [[ ! -d "$BUILD_DIR" ]]; then
    log_info "Setting target to $TARGET..."
    idf.py -B "$BUILD_DIR" set-target "$TARGET"
fi

# Build
log_info "Building $TYPE firmware v$VERSION..."
idf.py -B "$BUILD_DIR" build \
    -DFIRMWARE_TYPE="$FIRMWARE_TYPE" \
    -DFIRMWARE_VERSION_MAJOR="$MAJOR" \
    -DFIRMWARE_VERSION_MINOR="$MINOR" \
    -DFIRMWARE_VERSION_PATCH="$PATCH"

# Copy and rename output binary
if [[ -f "$BUILD_DIR/esp32-can-als.bin" ]]; then
    cp "$BUILD_DIR/esp32-can-als.bin" "$BUILD_DIR/$OUTPUT_NAME"
    log_success "Output: $BUILD_DIR/$OUTPUT_NAME"
else
    log_error "Build output not found!"
    exit 1
fi

# Show size if requested
if $DO_SIZE; then
    log_info "Size analysis:"
    idf.py -B "$BUILD_DIR" size
    echo ""
    idf.py -B "$BUILD_DIR" size-components | head -20
fi

# Get binary size
BIN_SIZE=$(stat -f%z "$BUILD_DIR/$OUTPUT_NAME" 2>/dev/null || stat -c%s "$BUILD_DIR/$OUTPUT_NAME" 2>/dev/null)
BIN_SIZE_KB=$((BIN_SIZE / 1024))

if [[ "$TYPE" == "factory" ]]; then
    MAX_SIZE=$((128 * 1024))
    MAX_SIZE_KB=128
else
    MAX_SIZE=$((1792 * 1024))  # 1.75MB
    MAX_SIZE_KB=1792
fi

echo ""
if [[ $BIN_SIZE -gt $MAX_SIZE ]]; then
    log_error "Binary size ($BIN_SIZE_KB KB) exceeds partition limit ($MAX_SIZE_KB KB)!"
    exit 1
else
    PERCENT=$((BIN_SIZE * 100 / MAX_SIZE))
    log_success "Binary size: $BIN_SIZE_KB KB / $MAX_SIZE_KB KB ($PERCENT%)"
fi

# Flash if requested
if $DO_FLASH; then
    log_info "Flashing to $PORT..."
    idf.py -B "$BUILD_DIR" -p "$PORT" flash
    log_success "Flash complete!"
fi

# Monitor if requested
if $DO_MONITOR; then
    log_info "Starting monitor on $PORT..."
    idf.py -B "$BUILD_DIR" -p "$PORT" monitor
fi

echo ""
log_success "Build complete: $TYPE v$VERSION"
echo ""
```

### Build Script Usage Examples

```bash
# Make script executable
chmod +x build.sh

# Build factory firmware (default target esp32c6)
./build.sh --type=factory --version=1.0.0

# Build main firmware for esp32c3
./build.sh --type=main --version=2.0.1 --target=esp32c3

# Build and flash factory firmware
./build.sh --type=factory --version=1.0.0 --flash --port=/dev/ttyUSB0

# Full clean rebuild with size analysis
./build.sh --type=main --version=2.1.0 --fullclean --size

# Build with custom ESP-IDF path
./build.sh --type=main --version=2.0.0 --idfpath=/opt/esp-idf

# Build, flash, and monitor
./build.sh --type=main --version=2.0.1 --flash --monitor
```

---

## Configuration Dependency Chain

### FIRMWARE_TYPE → CONFIG_BME680_ENABLED

The factory firmware **automatically disables** BME680 through the sdkconfig:

```
FIRMWARE_TYPE=FACTORY
    ↓
sdkconfig.defaults.factory is used
    ↓
CONFIG_BME680_ENABLED=n (set in sdkconfig.defaults.factory)
    ↓
BME680 code excluded from build
```

This is enforced at two levels:
1. **sdkconfig level**: `CONFIG_BME680_ENABLED=n` in factory config
2. **CMake level**: bsec component not required for FACTORY build
3. **Code level**: `#if IS_MAIN_FIRMWARE` guards

---

## Future Enhancements

1. **Firmware signature verification** - Add signing for OTA images
2. **Compressed OTA** - Reduce transfer time for large MAIN firmware
3. **Delta updates** - Only transfer changed portions
4. **Remote diagnostics** - FACTORY can report boot failures via CAN
5. **Secure boot integration** - Full secure boot v2 with signed updates
