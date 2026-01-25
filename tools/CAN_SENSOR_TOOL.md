# CAN Sensor Tool - Design & Implementation

This document describes `can-sensor-tool`, a C++ CLI utility for managing ESP32 CAN sensor nodes.

## Overview

**Purpose:** Unified management tool for ESP32-based CAN sensor nodes, supporting:
- Real-time sensor monitoring
- Device control (start/stop/reboot/shutdown)
- Node ID configuration for multi-node networks
- Factory reset and diagnostics
- **A/B OTA firmware updates over CAN**

**Location:** `tools/can-sensor-tool.cpp` with `tools/CMakeLists.txt`

**Requirements:**
- C++17
- Linux SocketCAN API
- No external dependencies (minimal, self-contained)

## Quick Start

```bash
# Build
cd tools && mkdir build && cd build && cmake .. && make

# Basic commands
./can-sensor-tool info                    # Query device info
./can-sensor-tool monitor                 # Live sensor data
./can-sensor-tool update firmware.bin     # OTA firmware update
./can-sensor-tool discover                # Find all nodes on bus
```

## Node ID Addressing Scheme

Each sensor node has a configurable base address with 16 message IDs reserved:

| Node ID | Base Address | Message Range | OTA Cmd ID | OTA Resp ID |
|---------|--------------|---------------|------------|-------------|
| 0 | 0x0A0 | 0x0A0 - 0x0AF | 0x700 | 0x708 |
| 1 | 0x0B0 | 0x0B0 - 0x0BF | 0x710 | 0x718 |
| 2 | 0x0C0 | 0x0C0 - 0x0CF | 0x720 | 0x728 |
| 3 | 0x0D0 | 0x0D0 - 0x0DF | 0x730 | 0x738 |
| 4 | 0x0E0 | 0x0E0 - 0x0EF | 0x740 | 0x748 |
| 5 | 0x0F0 | 0x0F0 - 0x0FF | 0x750 | 0x758 |

**Maximum nodes:** 6 (using CAN ID range 0x0A0-0x0FF for sensors, 0x700-0x758 for OTA)

**Default:** Node ID 0 (base 0x0A0) - backward compatible with existing firmware

**Storage:** Node ID stored in ESP32 NVS, persists across reboots

## CLI Interface

### Global Options

| Option | Default | Description |
|--------|---------|-------------|
| `--interface=<iface>` | `can0` | CAN interface name |
| `--node-id=<id>` | `0` | Target node ID (0-5) |
| `--quiet` | off | Suppress status messages, output data only |
| `--help` | - | Show help message |
| `--version` | - | Show tool version |

### Commands

#### Control Commands
```bash
can-sensor-tool start                    # Start sensor transmission
can-sensor-tool stop                     # Stop sensor transmission
can-sensor-tool shutdown                 # Graceful shutdown (save state, stop TX)
can-sensor-tool reboot                   # Save state and reboot
can-sensor-tool factory-reset            # Clear calibration, reboot
```

#### Monitoring Commands
```bash
can-sensor-tool monitor                  # Live sensor data (human-readable)
can-sensor-tool monitor --format=json    # JSON lines output
can-sensor-tool monitor --format=csv     # CSV output
can-sensor-tool monitor --count=10       # Stop after N messages
can-sensor-tool monitor --timeout=30     # Stop after N seconds
```

#### Configuration Commands
```bash
can-sensor-tool set-node-id <new_id>     # Set node ID (0-5), triggers reboot
can-sensor-tool info                     # Query device info/version/partition
can-sensor-tool discover                 # Scan bus for all nodes
```

#### Firmware Update Commands
```bash
can-sensor-tool update <firmware.bin>    # Update node 0 firmware
can-sensor-tool --node-id=2 update fw.bin # Update node 2 firmware
can-sensor-tool update --verify-only fw.bin # Verify CRC without flashing
can-sensor-tool update --no-activate fw.bin # Write but don't activate
can-sensor-tool update --no-reboot fw.bin   # Activate but don't reboot
can-sensor-tool update --chunk-delay=10 fw.bin # Add delay between chunks
```

### Usage Examples

```bash
# Monitor node 0 on default interface
can-sensor-tool monitor

# Monitor node 2 on can1 interface
can-sensor-tool --interface=can1 --node-id=2 monitor

# Reboot node 1
can-sensor-tool --node-id=1 reboot

# Change node 0 to node 3 (will auto-reboot)
can-sensor-tool --node-id=0 set-node-id 3

# Discover all nodes on the bus
can-sensor-tool discover

# Get device info from node 0 (shows firmware version, partition, sensors)
can-sensor-tool info

# Monitor with JSON output for scripting
can-sensor-tool monitor --format=json --quiet

# Log to file with CSV format
can-sensor-tool monitor --format=csv --quiet > sensor_log.csv

# Update firmware on node 0
can-sensor-tool update build/esp32_can_counter.bin

# Update node 2 with verbose output
can-sensor-tool --node-id=2 update firmware.bin
```

## Device Info Command

The `info` command queries detailed device information including A/B partition status:

```bash
$ can-sensor-tool info
Device Info (Node 0):
  Firmware:    v1.0.1
  Partition:   ota_0 (valid)
  Base Addr:   0x0A0
  Sensors:     ALS(VEML7700) BME680
  TX Active:   Yes
```

### INFO_RESPONSE Message Format (0x0AD)

```
Byte 0: Current node ID
Byte 1: Firmware version major
Byte 2: Firmware version minor
Byte 3: Firmware version patch
Byte 4: Sensor flags (bit 0: ALS, bit 1: BME680, bit 2: LD2410, bit 3: MQ-3)
Byte 5: ALS type (0=none, 1=VEML7700, 2=OPT4001)
Byte 6: Status flags (bit 0: transmitting)
Byte 7: Partition info (bits 0-2: type, bits 4-6: OTA state)
```

**Partition Types (bits 0-2):**
| Value | Name | Description |
|-------|------|-------------|
| 0 | factory | Running from factory partition |
| 1 | ota_0 | Running from OTA slot A |
| 2 | ota_1 | Running from OTA slot B |
| 7 | unknown | Unknown partition |

**OTA Image States (bits 4-6):**
| Value | Name | Description |
|-------|------|-------------|
| 0 | undefined | Not an OTA partition or unknown |
| 1 | new | First boot after OTA update |
| 2 | pending | Pending verification |
| 3 | valid | Validated and confirmed |
| 4 | invalid | Invalid / rollback triggered |
| 5 | aborted | Update aborted |

## A/B Firmware Update (OTA)

### Overview

The A/B (dual-partition) firmware update mechanism allows safe over-the-air updates via CAN bus:
- **Two OTA partitions:** ota_0 and ota_1 (plus factory fallback)
- **Atomic updates:** New firmware written to inactive partition, then activated
- **Rollback support:** If new firmware fails validation, boot back to previous version
- **Per-node addressing:** Update specific nodes without affecting others
- **Fast transfers:** ~3 KB/s with default settings (~2 min for 350KB firmware)

### Update Command Options

| Option | Default | Description |
|--------|---------|-------------|
| `--no-activate` | off | Write firmware but don't activate |
| `--no-reboot` | off | Activate but don't reboot |
| `--verify-only` | off | Just verify file CRC, no upload |
| `--chunk-delay=<ms>` | 0 | Delay between chunks (0 = fastest) |

### Update Progress Display

```
$ can-sensor-tool update build/esp32_can_counter.bin
Reading firmware file: build/esp32_can_counter.bin (274032 bytes)
Firmware CRC32: 0xA1B2C3D4
Connecting to node 0 (OTA CAN ID: 0x700)...
Starting update (erasing partition)...
Partition ready (max_chunk=6, free_space=1048576)
Uploading firmware...
Uploading: [==============================] 100% (274032/274032) 3.0 KB/s
Upload complete, verifying CRC...
Firmware verified successfully.
Activating new firmware...
Update complete! Node 0 is rebooting to new firmware.
```

### A/B Partition Alternation

Updates alternate between ota_0 and ota_1:

```
Initial state:  factory (undefined)
After update 1: ota_0 (valid)
After update 2: ota_1 (valid)
After update 3: ota_0 (valid)
...
```

Verify with `info` command after each update to confirm partition switch.

### Protocol Timing

| Parameter | Value | Description |
|-----------|-------|-------------|
| ACK timeout (initial) | 50ms | First attempt timeout |
| ACK timeout (max) | 150ms | Maximum after retries |
| Retry backoff | Exponential | 50ms → 100ms → 150ms → 150ms |
| Max retries per chunk | 3 | Then abort update |
| Start timeout | 10s | Partition erase time |
| Finish timeout | 5s | CRC verification time |
| Chunk delay | 0ms | Inter-chunk delay (configurable) |
| Session timeout | 5 min | Auto-abort if idle |

### OTA CAN Protocol

OTA uses a separate ID range (0x700+) to avoid conflicts with sensor data.

**Commands (Tool → ESP32):**

| Type | Name | Payload | Description |
|------|------|---------|-------------|
| 0x01 | START_UPDATE | [Size:4][CRC16:2] | Begin update |
| 0x02 | SEND_CHUNK | [Seq:1][Data:6] | Firmware chunk |
| 0x03 | FINISH_UPDATE | [CRC32:4][Pad:2] | End update, verify CRC |
| 0x04 | ACTIVATE_FW | [Flags:1][Pad:5] | Set boot partition |
| 0x05 | GET_STATUS | [Pad:6] | Query OTA status |
| 0x06 | ABORT_UPDATE | [Pad:6] | Cancel update |

**Responses (ESP32 → Tool):**

| Type | Name | Payload | Description |
|------|------|---------|-------------|
| 0x81 | ACK | [Seq:1][Pad:5] | Chunk received OK |
| 0x82 | NAK | [Seq:1][ErrCode:1][Pad:4] | Error, retransmit |
| 0x83 | STATUS | [State:1][Progress:2][Err:1][Pad:2] | Current state |
| 0x84 | READY | [MaxChunk:2][FreeSpace:4] | Ready to receive |
| 0x85 | COMPLETE | [Result:1][Pad:5] | Update finished |

**Error Codes:**

| Code | Name | Description |
|------|------|-------------|
| 0x00 | OK | No error |
| 0x01 | ERR_BUSY | Update already in progress |
| 0x02 | ERR_NO_SPACE | Firmware too large |
| 0x03 | ERR_SEQ | Sequence mismatch |
| 0x04 | ERR_CRC | CRC verification failed |
| 0x05 | ERR_WRITE | Flash write error |
| 0x06 | ERR_INVALID | Invalid command/state |
| 0x07 | ERR_TIMEOUT | Operation timed out |
| 0x08 | ERR_ABORTED | Update aborted |

## CAN Protocol Reference

### Sensor Messages (per node, offset from base)

| Offset | ID (Node 0) | Direction | Description |
|--------|-------------|-----------|-------------|
| +0x00 | 0x0A0 | RX | STOP - Stop transmission |
| +0x01 | 0x0A1 | RX | START - Start transmission |
| +0x02 | 0x0A2 | TX | Ambient light data (1 Hz) |
| +0x03 | 0x0A3 | TX | Environmental data T/H/P (0.33 Hz) |
| +0x04 | 0x0A4 | TX | Air quality IAQ/CO2/VOC (0.33 Hz) |
| +0x07 | 0x0A7 | TX | System status (0.1 Hz) |
| +0x08 | 0x0A8 | RX | SHUTDOWN - Graceful shutdown |
| +0x09 | 0x0A9 | RX | REBOOT - Save state and reboot |
| +0x0A | 0x0AA | RX | FACTORY_RESET - Clear calibration |
| +0x0B | 0x0AB | RX | SET_NODE_ID - Set new node ID |
| +0x0C | 0x0AC | RX | GET_INFO - Request device info |
| +0x0D | 0x0AD | TX | INFO_RESPONSE - Device info |
| +0x0E | 0x0AE | RX | PING - Discovery ping |
| +0x0F | 0x0AF | TX | PONG - Discovery response |

### Discovery Protocol

1. Tool sends PING to each possible base address: 0x0AE, 0x0BE, 0x0CE, 0x0DE, 0x0EE, 0x0FE
2. Wait 300ms for responses
3. Nodes respond with PONG on their respective +0x0F offset
4. Tool collects and displays all responding nodes

## ESP32 Firmware Version

### Default Version

The firmware version is defined in `main/can_protocol.h`:

```c
#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0
```

### Override via Command Line

Override firmware version when building:

```bash
# Override patch version
idf.py build -DCMAKE_C_FLAGS="-DFIRMWARE_VERSION_PATCH=1"

# Override multiple components
idf.py build -DCMAKE_C_FLAGS="-DFIRMWARE_VERSION_MINOR=1 -DFIRMWARE_VERSION_PATCH=0"

# Full version override
idf.py build -DCMAKE_C_FLAGS="-DFIRMWARE_VERSION_MAJOR=2 -DFIRMWARE_VERSION_MINOR=0 -DFIRMWARE_VERSION_PATCH=0"
```

Verify version after flashing:
```bash
$ can-sensor-tool info
Device Info (Node 0):
  Firmware:    v1.0.1
  ...
```

## ESP32 Partition Table

Required partition layout for A/B OTA (in `partitions.csv`):

```
# Name,   Type, SubType,  Offset,   Size
nvs,      data, nvs,      0x9000,   0x6000
otadata,  data, ota,      0xf000,   0x2000
phy_init, data, phy,      0x11000,  0x1000
ota_0,    app,  ota_0,    0x20000,  0x100000
ota_1,    app,  ota_1,    0x120000, 0x100000
```

**Partition sizes:**
- ota_0: 1 MB (primary OTA slot)
- ota_1: 1 MB (secondary OTA slot)
- otadata: 8 KB (boot selection data)

## Implementation Status

### Completed Features

- [x] **Phase 1: Infrastructure** - CLI parsing, CAN socket, help/version
- [x] **Phase 2: Control Commands** - start/stop/shutdown/reboot/factory-reset
- [x] **Phase 3: Monitor Command** - Human/JSON/CSV output, count/timeout options
- [x] **Phase 4: Node ID Management** - set-node-id, info, discover commands
- [x] **Phase 5: ESP32 Firmware** - NVS storage, node ID offset, GET_INFO/PING handlers
- [x] **Phase 6: Calibration** - Deferred (existing calibrate_lux_sensor tool)
- [x] **Phase 7: A/B OTA Update** - Full implementation complete
  - [x] ESP32 partition table for A/B OTA
  - [x] OTA handler state machine
  - [x] Chunked transfer with ACK/NAK
  - [x] CRC32 verification
  - [x] Partition activation and reboot
  - [x] Rollback prevention (mark valid on boot)
  - [x] Tool update command with progress display
  - [x] Exponential backoff for retries
  - [x] Partition info in INFO_RESPONSE

### Testing Checklist

#### Tool Build & Basic Function
- [x] `cmake .. && make` succeeds
- [x] `./can-sensor-tool --help` shows usage
- [x] `./can-sensor-tool --version` shows version

#### Control Commands
- [x] `start` - ESP32 begins transmitting
- [x] `stop` - ESP32 stops transmitting
- [x] `shutdown` - ESP32 saves state, stops
- [x] `reboot` - ESP32 reboots
- [x] `factory-reset` - ESP32 clears calibration

#### Monitor Command
- [x] Human-readable output displays correctly
- [x] JSON output is valid JSON
- [x] CSV output has correct headers and values
- [x] `--count` stops after N messages
- [x] `--timeout` stops after N seconds
- [x] `--quiet` suppresses status messages

#### Multi-Node
- [x] `set-node-id` changes node ID and reboots
- [x] `info` returns correct device information
- [x] `info` shows partition type and OTA state
- [x] `discover` finds all active nodes
- [x] Commands with `--node-id=N` target correct node

#### OTA Firmware Update
- [x] `update` reads and validates firmware file
- [x] `--verify-only` checks CRC without flashing
- [x] START_UPDATE erases target partition
- [x] Chunked transfer with ACK per chunk
- [x] Progress bar displays correctly (~3 KB/s)
- [x] CRC32 verification at end
- [x] Partition activation works
- [x] ESP32 reboots to new firmware
- [x] A/B alternation (ota_0 → ota_1 → ota_0)
- [x] `info` shows new partition and "valid" state
- [x] `--no-activate` writes without activating
- [x] `--no-reboot` activates without rebooting
- [x] `--chunk-delay=N` adds inter-chunk delay
- [x] Exponential backoff on retries (50→100→150ms)

## File Structure

```
tools/
├── CMakeLists.txt
├── can-sensor-tool.cpp
├── build/
│   └── can-sensor-tool (executable)
└── CAN_SENSOR_TOOL.md (this file)

main/
├── can_protocol.h     # Protocol definitions, version, OTA constants
├── can_protocol.c     # Message formatting functions
├── ota_handler.h      # OTA handler interface
├── ota_handler.c      # OTA state machine and CAN handlers
└── main.c             # Main app with OTA integration
```

## Build Instructions

```bash
# Build tool
cd tools
mkdir build && cd build
cmake ..
make

# Install (optional)
sudo cp can-sensor-tool /usr/local/bin/

# Build ESP32 firmware
cd ../..  # Back to project root
idf.py build

# Flash ESP32
idf.py -p /dev/ttyUSB0 flash monitor
```

## Dependencies

**Tool:**
- Linux kernel with SocketCAN support
- C++17 compiler (g++ 7+ or clang++ 5+)
- CMake 3.10+
- No external libraries required

**ESP32 Firmware:**
- ESP-IDF 5.x
- esp_ota_ops component
- esp_partition component
- nvs_flash component

## Troubleshooting

### OTA Update Fails at Start
- Ensure ESP32 has A/B partition table flashed
- Check CAN bus connectivity with `candump can0`
- Verify node ID matches: `can-sensor-tool info`

### Slow Update Speed
- Default is already optimized (0ms chunk delay)
- ~3 KB/s is expected for standard CAN (500 kbps, 6 bytes/frame with ACK)
- For faster updates, consider CAN FD (future enhancement)

### Update Succeeds but Old Firmware Runs
- Check `info` output for partition state
- If "pending", firmware may have failed validation
- If "invalid", rollback occurred - check firmware integrity

### "Sequence Error" During Update
- Usually recovers with automatic retry
- If persistent, check for CAN bus errors/noise
- Try adding `--chunk-delay=5` for slower but more reliable transfer
