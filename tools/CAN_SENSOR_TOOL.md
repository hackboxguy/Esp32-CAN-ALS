# CAN Sensor Tool - Design & Implementation Plan

This document outlines the design, implementation plan, and progress tracking for `can-sensor-tool`, a C++ CLI utility for managing ESP32 CAN sensor nodes.

## Overview

**Purpose:** Unified management tool for ESP32-based CAN sensor nodes, supporting:
- Real-time sensor monitoring
- Device control (start/stop/reboot/shutdown)
- Node ID configuration for multi-node networks
- Factory reset and diagnostics
- Future: calibration, firmware updates

**Location:** `tools/can-sensor-tool.cpp` with `tools/CMakeLists.txt`

**Requirements:**
- C++17
- Linux SocketCAN API
- No external dependencies (minimal, self-contained)

## Node ID Addressing Scheme

Each sensor node has a configurable base address with 16 message IDs reserved:

| Node ID | Base Address | Message Range |
|---------|--------------|---------------|
| 0 | 0x0A0 | 0x0A0 - 0x0AF |
| 1 | 0x0B0 | 0x0B0 - 0x0BF |
| 2 | 0x0C0 | 0x0C0 - 0x0CF |
| 3 | 0x0D0 | 0x0D0 - 0x0DF |
| 4 | 0x0E0 | 0x0E0 - 0x0EF |
| 5 | 0x0F0 | 0x0F0 - 0x0FF |

**Maximum nodes:** 6 (using CAN ID range 0x0A0-0x0FF)

**Default:** Node ID 0 (base 0x0A0) - backward compatible with existing firmware

**Storage:** Node ID stored in ESP32 NVS, persists across reboots

## CLI Interface Specification

### Global Options

| Option | Default | Description |
|--------|---------|-------------|
| `--interface=<iface>` | `can0` | CAN interface name |
| `--speed=<bps>` | `500000` | CAN bus speed (for reference/validation) |
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
can-sensor-tool info                     # Query device info/version
can-sensor-tool discover                 # Scan bus for all nodes
```

#### Future Commands (Placeholder)
```bash
can-sensor-tool calibrate [options]      # Lux sensor calibration
can-sensor-tool update <firmware.bin>    # Firmware update over CAN
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

# Get device info from node 0
can-sensor-tool info

# Monitor with JSON output for scripting
can-sensor-tool monitor --format=json --quiet

# Log to file with CSV format
can-sensor-tool monitor --format=csv --quiet > sensor_log.csv
```

## CAN Protocol Extensions

### Existing Messages (per node, offset from base)

| Offset | ID (Node 0) | Direction | Description |
|--------|-------------|-----------|-------------|
| +0x00 | 0x0A0 | RX | STOP - Stop transmission |
| +0x01 | 0x0A1 | RX | START - Start transmission |
| +0x02 | 0x0A2 | TX | Ambient light data (1 Hz) |
| +0x03 | 0x0A3 | TX | Environmental data T/H/P (0.33 Hz) |
| +0x04 | 0x0A4 | TX | Air quality IAQ/CO2/VOC (0.33 Hz) |
| +0x08 | 0x0A8 | RX | SHUTDOWN - Graceful shutdown |
| +0x09 | 0x0A9 | RX | REBOOT - Save state and reboot |
| +0x0A | 0x0AA | RX | FACTORY_RESET - Clear calibration |

### New Messages (per node, offset from base)

| Offset | ID (Node 0) | Direction | Description |
|--------|-------------|-----------|-------------|
| +0x0B | 0x0AB | RX | SET_NODE_ID - Set new node ID |
| +0x0C | 0x0AC | RX | GET_INFO - Request device info |
| +0x0D | 0x0AD | TX | INFO_RESPONSE - Device info response |
| +0x0E | 0x0AE | RX | PING - Discovery ping |
| +0x0F | 0x0AF | TX | PONG - Discovery response |

### Message Formats

#### SET_NODE_ID (0x0AB) - Tool → ESP32
```
Byte 0: New node ID (0-5)
Byte 1-7: Reserved (0x00)
```
**Behavior:** ESP32 saves new node ID to NVS and auto-reboots

#### GET_INFO (0x0AC) - Tool → ESP32
```
Byte 0-7: Reserved (0x00) - empty request
```

#### INFO_RESPONSE (0x0AD) - ESP32 → Tool
```
Byte 0: Current node ID
Byte 1: Firmware version major
Byte 2: Firmware version minor
Byte 3: Firmware version patch
Byte 4: Sensor flags (bit 0: ALS, bit 1: BME680, bit 2-7: reserved)
Byte 5: ALS type (0=none, 1=VEML7700, 2=OPT4001)
Byte 6: Status flags (bit 0: transmitting, bit 1-7: reserved)
Byte 7: Reserved
```

#### PING (0x0AE) - Tool → ESP32
```
Byte 0-7: Reserved (0x00) - empty ping
```

#### PONG (0x0AF) - ESP32 → Tool
```
Byte 0: Current node ID
Byte 1-7: Reserved (0x00)
```

### Discovery Protocol

1. Tool sends PING to each possible base address: 0x0AE, 0x0BE, 0x0CE, 0x0DE, 0x0EE, 0x0FE
2. Wait 100ms for responses
3. Nodes respond with PONG on their respective +0x0F offset
4. Tool collects and displays all responding nodes

## Implementation Phases

### Phase 1: Infrastructure
- [x] Create `tools/` directory structure
- [x] Create `tools/CMakeLists.txt`
- [x] Implement basic CLI argument parsing (hand-rolled, no deps)
- [x] Implement CAN socket initialization (SocketCAN)
- [x] Implement CAN send/receive helper functions
- [x] Add `--help` and `--version` support
- [x] Test: Build and verify help output

### Phase 2: Simple Control Commands
- [x] Implement `start` command (send 0x0A1)
- [x] Implement `stop` command (send 0x0A0)
- [x] Implement `shutdown` command (send 0x0A8)
- [x] Implement `reboot` command (send 0x0A9)
- [x] Implement `factory-reset` command (send 0x0AA)
- [x] Add node ID offset calculation for all commands
- [x] Test: Verify commands work with existing ESP32 firmware

### Phase 3: Monitor Command
- [x] Implement CAN message receive loop
- [x] Parse ambient light message (0x0A2)
- [x] Parse environmental message (0x0A3)
- [x] Parse air quality message (0x0A4)
- [x] Implement human-readable output format
- [x] Implement JSON output format (`--format=json`)
- [x] Implement CSV output format (`--format=csv`)
- [x] Add `--count` option (stop after N messages)
- [x] Add `--timeout` option (stop after N seconds)
- [x] Add `--quiet` mode support
- [x] Test: Monitor existing ESP32 sensor output

### Phase 4: Node ID Management (Tool Side)
- [x] Implement `set-node-id` command
- [x] Implement `info` command (send GET_INFO, parse response)
- [x] Implement `discover` command (ping all bases, collect responses)
- [x] Add validation for node ID range (0-5)
- [x] Test: Commands send correct CAN messages (verified with candump)

### Phase 5: ESP32 Firmware Changes
- [x] Add NVS storage for node ID
- [x] Load node ID from NVS at boot (default 0 if not set)
- [x] Apply node ID offset to all CAN message IDs
- [x] Implement SET_NODE_ID handler (save to NVS, reboot)
- [x] Implement GET_INFO handler (respond with device info)
- [x] Implement PING/PONG handler for discovery
- [x] Add firmware version constants
- [x] Test: Full integration with can-sensor-tool

### Phase 6: Calibration (Future)
- [ ] Design calibration CAN protocol
- [ ] Implement `calibrate` command
- [ ] Port functionality from existing calibrate_lux_sensor tool
- [ ] Test: End-to-end calibration workflow

### Phase 7: Firmware Update (A/B OTA over CAN)
- [ ] Update ESP32 partition table for A/B OTA
- [ ] Implement OTA CAN message handlers in ESP32 firmware
- [ ] Implement `update` command in can-sensor-tool
- [ ] Test: Full firmware update cycle

---

## A/B Firmware Update Design

### Overview

The A/B (dual-partition) firmware update mechanism allows safe over-the-air updates via CAN bus:
- **Two OTA partitions:** ota_0 and ota_1 (plus factory fallback)
- **Atomic updates:** New firmware written to inactive partition, then activated
- **Rollback support:** If new firmware fails, boot back to previous version
- **Per-node addressing:** Update specific nodes without affecting others

### CAN ID Allocation for OTA

OTA uses a separate ID range (0x700+) to avoid conflicts with sensor data:

| Direction | Base ID | Per-Node Offset | Description |
|-----------|---------|-----------------|-------------|
| Tool → ESP32 | 0x700 | +0x10 per node | OTA Commands |
| ESP32 → Tool | 0x708 | +0x10 per node | OTA Responses |

**Node-specific IDs:**
| Node ID | Command ID | Response ID |
|---------|------------|-------------|
| 0 | 0x700 | 0x708 |
| 1 | 0x710 | 0x718 |
| 2 | 0x720 | 0x728 |
| 3 | 0x730 | 0x738 |
| 4 | 0x740 | 0x748 |
| 5 | 0x750 | 0x758 |

### Custom Multi-Frame Protocol

Since CAN frames are limited to 8 bytes, firmware data is sent in chunks with ACK-based reliability.

**Frame Format (8 bytes):**
```
Byte 0:    Command/Response type
Byte 1:    Sequence number (0-255, wraps)
Byte 2-7:  Payload (6 bytes)
```

### OTA Commands (Tool → ESP32)

| Type | Name | Payload | Description |
|------|------|---------|-------------|
| 0x01 | START_UPDATE | [Size:4][CRC32:2] | Begin update, total size + truncated CRC |
| 0x02 | SEND_CHUNK | [Data:6] | Firmware data chunk (6 bytes) |
| 0x03 | FINISH_UPDATE | [CRC32:4][Pad:2] | End update, verify full CRC32 |
| 0x04 | ACTIVATE_FW | [Flags:1][Pad:5] | Set boot partition, optionally reboot |
| 0x05 | GET_STATUS | [Pad:6] | Query current OTA status |
| 0x06 | ABORT_UPDATE | [Pad:6] | Cancel in-progress update |

### OTA Responses (ESP32 → Tool)

| Type | Name | Payload | Description |
|------|------|---------|-------------|
| 0x81 | ACK | [SeqNum:1][Pad:5] | Chunk received OK |
| 0x82 | NAK | [SeqNum:1][ErrCode:1][Pad:4] | Error, request retransmit |
| 0x83 | STATUS | [State:1][Progress:2][Err:1][Pad:2] | Current OTA state |
| 0x84 | READY | [MaxChunk:2][FreeSpace:4] | Ready to receive update |
| 0x85 | COMPLETE | [Result:1][Pad:5] | Update finished (success/fail) |

### Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0x00 | OK | No error |
| 0x01 | ERR_BUSY | Update already in progress |
| 0x02 | ERR_NO_SPACE | Firmware too large for partition |
| 0x03 | ERR_SEQ | Sequence number mismatch |
| 0x04 | ERR_CRC | CRC verification failed |
| 0x05 | ERR_WRITE | Flash write error |
| 0x06 | ERR_INVALID | Invalid command or state |
| 0x07 | ERR_TIMEOUT | Operation timed out |
| 0x08 | ERR_ABORTED | Update was aborted |

### OTA State Machine (ESP32)

```
                    ┌─────────────┐
                    │    IDLE     │
                    └──────┬──────┘
                           │ START_UPDATE
                           ▼
                    ┌─────────────┐
         ┌─────────│  RECEIVING  │◄────────┐
         │         └──────┬──────┘         │
         │ ABORT          │ SEND_CHUNK     │ NAK (retry)
         │                ▼                │
         │         ┌─────────────┐         │
         │         │  VERIFYING  │─────────┘
         │         └──────┬──────┘
         │                │ FINISH_UPDATE (CRC OK)
         │                ▼
         │         ┌─────────────┐
         │         │   READY     │
         │         └──────┬──────┘
         │                │ ACTIVATE_FW
         │                ▼
         │         ┌─────────────┐
         └────────►│    IDLE     │ (or REBOOTING)
                   └─────────────┘
```

### Update Protocol Flow

```
Tool                                    ESP32
 │                                        │
 │──── START_UPDATE [size, crc16] ───────►│
 │                                        │ (Erase partition)
 │◄──── READY [max_chunk, free_space] ───│
 │                                        │
 │──── SEND_CHUNK [seq=0, data] ─────────►│
 │◄──── ACK [seq=0] ─────────────────────│
 │                                        │
 │──── SEND_CHUNK [seq=1, data] ─────────►│
 │◄──── ACK [seq=1] ─────────────────────│
 │                                        │
 │           ... repeat ...               │
 │                                        │
 │──── SEND_CHUNK [seq=N, data] ─────────►│
 │◄──── ACK [seq=N] ─────────────────────│
 │                                        │
 │──── FINISH_UPDATE [crc32] ────────────►│
 │                                        │ (Verify CRC)
 │◄──── COMPLETE [result=OK] ────────────│
 │                                        │
 │──── ACTIVATE_FW [reboot=1] ───────────►│
 │◄──── ACK ─────────────────────────────│
 │                                        │ (Reboot)
```

### Retry and Timeout Handling

- **Chunk ACK timeout:** 500ms (retry up to 3 times)
- **Start/Finish timeout:** 5 seconds (partition erase takes time)
- **Inter-chunk delay:** 10ms minimum (allow flash writes)
- **Max retries per chunk:** 3 (then abort)
- **Session timeout:** 5 minutes (auto-abort if idle)

### ESP32 Partition Table

Required partition layout for A/B OTA:

```
# Name,   Type, SubType,  Offset,   Size
nvs,      data, nvs,      0x9000,   0x6000
phy_init, data, phy,      0xf000,   0x1000
otadata,  data, ota,      0x10000,  0x2000
factory,  app,  factory,  0x20000,  0x100000
ota_0,    app,  ota_0,    0x120000, 0x100000
ota_1,    app,  ota_1,    0x220000, 0x100000
```

**Partition sizes:**
- factory: 1 MB (fallback firmware)
- ota_0: 1 MB (primary OTA slot)
- ota_1: 1 MB (secondary OTA slot)
- otadata: 8 KB (boot selection data)

### ESP32 Firmware Changes

**New files:**
- `main/ota_handler.c/h` - OTA state machine and CAN handlers

**Modified files:**
- `main/main.c` - Add OTA task, register OTA CAN message handlers
- `main/can_protocol.h` - Add OTA message ID definitions
- `partitions.csv` - A/B partition layout
- `CMakeLists.txt` - Include OTA handler

**Key ESP-IDF APIs:**
```c
#include "esp_ota_ops.h"
#include "esp_partition.h"

// Get next OTA partition to write
esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

// Begin OTA update
esp_ota_handle_t ota_handle;
esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);

// Write firmware data
esp_ota_write(ota_handle, chunk_data, chunk_size);

// Finish and verify
esp_ota_end(ota_handle);

// Activate new firmware
esp_ota_set_boot_partition(update_partition);

// Reboot
esp_restart();
```

### Tool-Side Implementation

**New command:**
```bash
can-sensor-tool update <firmware.bin>           # Update node 0
can-sensor-tool --node-id=2 update firmware.bin # Update node 2
can-sensor-tool update --verify-only fw.bin     # Verify CRC without flashing
```

**Update command options:**
| Option | Default | Description |
|--------|---------|-------------|
| `--no-activate` | off | Write firmware but don't activate |
| `--no-reboot` | off | Activate but don't reboot |
| `--verify-only` | off | Just verify file CRC |
| `--chunk-delay=<ms>` | 10 | Delay between chunks |
| `--timeout=<sec>` | 300 | Total update timeout |

**Progress display:**
```
$ can-sensor-tool --node-id=0 update firmware.bin
Reading firmware file: firmware.bin (294912 bytes)
Firmware CRC32: 0xA1B2C3D4
Connecting to node 0...
Starting update (partition: ota_0)...
Erasing partition... done
Uploading: [████████████████████░░░░] 85% (250880/294912) 48 KB/s
Verifying CRC... OK
Activating new firmware...
Rebooting node...
Update complete! Node 0 now running firmware v1.1.0
```

### Implementation Phases

#### Phase 7a: ESP32 Partition Setup
- [ ] Create `partitions.csv` with A/B layout
- [ ] Update `sdkconfig` for custom partition table
- [ ] Verify partition layout with `idf.py partition-table`
- [ ] Test: Boot factory firmware, verify OTA partitions visible

#### Phase 7b: ESP32 OTA Handler (Basic)
- [ ] Create `ota_handler.c/h` with state machine
- [ ] Add OTA CAN ID definitions to `can_protocol.h`
- [ ] Implement START_UPDATE handler (erase partition)
- [ ] Implement SEND_CHUNK handler (write to flash)
- [ ] Implement FINISH_UPDATE handler (verify CRC)
- [ ] Test: Receive firmware via CAN, verify written correctly

#### Phase 7c: ESP32 OTA Handler (Activation)
- [ ] Implement ACTIVATE_FW handler
- [ ] Implement GET_STATUS handler
- [ ] Implement ABORT_UPDATE handler
- [ ] Add rollback detection (mark firmware valid after successful boot)
- [ ] Test: Full update cycle with activation and reboot

#### Phase 7d: Tool Update Command
- [ ] Implement firmware file reading and CRC calculation
- [ ] Implement START_UPDATE with size/CRC
- [ ] Implement chunked transfer with ACK handling
- [ ] Implement retry logic and timeout handling
- [ ] Implement progress display
- [ ] Test: Update node 0, verify new firmware runs

#### Phase 7e: Multi-Node and Robustness
- [ ] Test updates to different node IDs
- [ ] Test recovery from interrupted update
- [ ] Test rollback after bad firmware
- [ ] Add --verify-only option
- [ ] Add --no-activate and --no-reboot options
- [ ] Test: Full integration with all options

### OTA Testing Checklist

#### Partition Setup
- [ ] Partition table compiled without errors
- [ ] `idf.py partition-table` shows correct layout
- [ ] Factory firmware boots correctly
- [ ] OTA partitions visible via `esp_partition_find()`

#### Basic Update Flow
- [ ] START_UPDATE erases target partition
- [ ] SEND_CHUNK writes data correctly
- [ ] Sequence numbers tracked properly
- [ ] FINISH_UPDATE verifies CRC32
- [ ] COMPLETE response sent with correct result

#### Activation and Reboot
- [ ] ACTIVATE_FW sets boot partition
- [ ] ESP32 reboots to new firmware
- [ ] New firmware version reported via `info` command
- [ ] GET_STATUS returns correct state

#### Error Handling
- [ ] NAK sent for sequence mismatch
- [ ] Retry after timeout works
- [ ] ABORT_UPDATE cleans up properly
- [ ] Large firmware (>512KB) handled correctly
- [ ] Invalid CRC rejected

#### Rollback Support
- [ ] Bad firmware triggers rollback
- [ ] Rollback to previous working firmware
- [ ] Factory reset boots factory partition

#### Tool Integration
- [ ] Progress bar displays correctly
- [ ] Timeout handling works
- [ ] --verify-only checks CRC without flashing
- [ ] --node-id targets correct node
- [ ] Update speed: target ~10 KB/s minimum

## ESP32 Firmware Changes Summary

Changes implemented to support can-sensor-tool:

### Modified Files

**can_protocol.h:**
- Added firmware version constants (`FIRMWARE_VERSION_MAJOR/MINOR/PATCH`)
- Added node ID addressing macros (`CAN_BASE_ADDR()`, `CAN_MSG_ID()`, etc.)
- Added message offset definitions for all 16 message types
- Added sensor flags and ALS type constants
- Added new function declarations for INFO_RESPONSE and PONG messages

**can_protocol.c:**
- Added `can_format_info_response()` function
- Added `can_format_pong_response()` function
- Added `can_set_msg_id()` helper to apply node ID offset

**main.c:**
- Added NVS namespace/key for node ID storage
- Added `g_node_id` and `g_tx_active` global variables
- Added `load_node_id_from_nvs()` and `save_node_id_to_nvs()` functions
- Added `get_sensor_flags()` and `get_als_type()` helper functions
- Updated `twai_receive_task()` to use node-ID-based CAN IDs
- Added handlers for SET_NODE_ID, GET_INFO, and PING commands
- Updated `twai_transmit_task()` to apply node ID offset to all TX messages
- Updated `app_main()` to load node ID from NVS at boot

**Example node ID addressing:**
```c
// can_protocol.h
#define CAN_BASE_ADDR(node_id)  (0x0A0 + ((node_id) * 0x10))
#define CAN_MSG_ID(node_id, offset)  (CAN_BASE_ADDR(node_id) + (offset))

#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0
```

## Testing Checklist

### Tool Build & Basic Function
- [x] `cmake .. && make` succeeds
- [x] `./can-sensor-tool --help` shows usage
- [x] `./can-sensor-tool --version` shows version

### Control Commands (with existing firmware, node 0 only)
- [x] `start` - ESP32 begins transmitting
- [x] `stop` - ESP32 stops transmitting
- [x] `shutdown` - ESP32 saves state, stops
- [x] `reboot` - ESP32 reboots
- [x] `factory-reset` - ESP32 clears calibration

### Monitor Command
- [x] Human-readable output displays correctly
- [x] JSON output is valid JSON
- [x] CSV output has correct headers and values
- [x] `--count` stops after N messages
- [x] `--timeout` stops after N seconds
- [x] `--quiet` suppresses status messages

### Multi-Node (firmware and tool implemented)
- [x] `set-node-id` changes node ID and reboots
- [x] `info` returns correct device information
- [x] `discover` finds all active nodes
- [x] Commands with `--node-id=N` target correct node
- [ ] Multiple nodes coexist on same bus (requires 2+ physical nodes)

## File Structure

```
tools/
├── CMakeLists.txt
├── can-sensor-tool.cpp
└── CAN_SENSOR_TOOL.md (this file)
```

## Build Instructions

```bash
cd tools
mkdir build && cd build
cmake ..
make

# Install (optional)
sudo make install
```

## Dependencies

- Linux kernel with SocketCAN support
- C++17 compiler (g++ 7+ or clang++ 5+)
- CMake 3.10+

No external libraries required.
