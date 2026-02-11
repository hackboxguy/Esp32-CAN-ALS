# ESP32 CAN Multi-Sensor Node

A professional-grade automotive sensor node for CAN networks using ESP32 with multi-sensor support, A/B firmware updates, and comprehensive node management.

## Overview

This project implements a multi-sensor node for automotive CAN networks using:
- **ESP32 microcontroller** (C6, C3, or other variants with CAN support)
- **Multi-sensor support** with runtime auto-detection:
  - **Ambient Light:** VEML7700 (0-120K lux) OR OPT3001/OPT4001 (auto-range)
  - **Environmental:** BME680/BME688 (temperature, humidity, pressure, air quality)
- **SN65HVD230 CAN transceiver** for robust automotive communication
- **A/B OTA firmware updates** over CAN bus (~3 KB/s)
- **Multi-node support** with configurable node IDs (0-15)

## Features

- **Multi-sensor support** with runtime auto-detection
  - VEML7700: 0-120K lux with intelligent auto-ranging
  - OPT3001: 0-83K lux, factory calibrated
  - OPT4001: 0-2.2M lux, factory calibrated
  - BME680/BME688: Temperature, humidity, pressure, IAQ/CO2/VOC
- **A/B firmware updates** over CAN bus with rollback support
- **Multi-node addressing** (up to 16 nodes on same CAN bus)
- **Comprehensive node management** via can-sensor-tool:
  - Device discovery and monitoring
  - Remote reboot and factory reset
  - Node ID configuration
  - OTA firmware updates
- **Automotive CAN bus** integration (500 kbps)
- **Structured 8-byte CAN messages** with checksums

## Hardware Requirements

### Core Components
- ESP32 development board (ESP32-C6, ESP32-C3, or compatible)
- **Choose ambient light sensor (optional):**
  - VEML7700 (I2C address 0x10), OR
  - OPT3001 / OPT4001 (I2C address 0x44, auto-detected)
- **Environmental sensor (optional):**
  - BME680 or BME688 (I2C address 0x76 or 0x77)
- SN65HVD230 CAN transceiver breakout board
- CAN-to-USB adapter (for development/testing)

**Notes:**
- The firmware automatically detects which sensors are connected at runtime.
- The current sensor PCB footprint supports **either** VEML7700 or OPT3001 (mount one). If both are mounted, VEML7700 is ignored.
- The sensor and monitor boards include a buck converter (6-30V input, 12V typical) and provide stable 5V/3.3V locally. With a two-pair 6P4C cable, this functions as a simple “power-over-CAN” setup (one pair for CAN, one pair for power).

### Wiring Diagram

```
ESP32          Sensors (I2C)            SN65HVD230          CAN Bus
-----          -------------            -------             -------
GPIO6 (SDA)    <--> SDA (all sensors)
GPIO7 (SCL)    <--> SCL (all sensors)
3.3V           ---> VCC (all sensors)
GND            ---> GND (all sensors)

GPIO4 (CAN_TX) ---> TX
GPIO5 (CAN_RX) <--- RX
3.3V           ---> VCC
GND            ---> GND                 GND <-----------> GND
                                       CANH -----------> CAN_HIGH
                                       CANL -----------> CAN_LOW
```

**Important Notes:**
- SN65HVD230 is 3.3V native (no level shifting needed with ESP32-C3)
- Multiple I2C sensors share the same bus (different addresses)
- Only ONE ambient light sensor at a time (VEML7700 OR OPT3001/OPT4001)

## Software Prerequisites

### ESP-IDF Installation
```bash
# Install dependencies (Ubuntu/Debian)
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dkms libusb-1.0-0

# Clone ESP-IDF
mkdir ~/esp && cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git

# Install ESP-IDF toolchain for your target chip(s)
cd ~/esp/esp-idf
./install.sh esp32c3          # for ESP32-C3
# or: ./install.sh esp32c6    # for ESP32-C6
# or: ./install.sh esp32c3,esp32c6  # for both

# Activate ESP-IDF environment (required in every new terminal)
. ~/esp/esp-idf/export.sh

# Verify it worked
idf.py --version
```

**Important notes:**
- `export.sh` requires **bash** (not `sh`/`dash`). Use `bash` as your shell or run `. ~/esp/esp-idf/export.sh` from bash.
- If `export.sh` fails with Python dependency errors (e.g., missing `tree_sitter`), re-run `./install.sh` for your target to update the Python virtual environment.
- After upgrading ESP-IDF (e.g., `git pull`), always re-run `./install.sh` before building.

### Linux CAN Utilities
```bash
# Install can-utils
sudo apt-get install can-utils

# Setup CAN interface (500 kbps)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

## Building and Flashing

### ESP32 Firmware

Use the provided `build.sh` script for simplified builds (**requires bash**):

```bash
git clone https://github.com/hackboxguy/Esp32-CAN-ALS.git
cd Esp32-CAN-ALS/

# First build (specify target)
./build.sh --target=esp32c3 --version=1.0.0

# If ESP-IDF is not in ~/esp/esp-idf, specify the path
./build.sh --target=esp32c3 --idfpath=/path/to/esp-idf

# Rebuild (target remembered)
./build.sh

# Build and flash
./build.sh --target=esp32c3 --flash --port=/dev/ttyACM0

# Flash existing binary without rebuilding
./build.sh --flash-only --port=/dev/ttyACM0

# Clean build
./build.sh --clean --target=esp32c3

# With BSEC library for BME680/688 support
./build.sh --target=esp32c3 --bsecpath=bsec2-6-1-0_generic_release_22102024.zip
```

**Note:** Always run `build.sh` with bash (`./build.sh` or `bash build.sh`), not `sh build.sh`.

Or use idf.py directly (after sourcing `export.sh`):
```bash
. ~/esp/esp-idf/export.sh
idf.py set-target esp32c3
idf.py build flash monitor
```

### CAN Sensor Tool
```bash
cd tools/
make

# Install system-wide (optional)
sudo make install
```

## Usage

### Basic Operation

Once flashed, the ESP32 automatically:
1. Auto-detects connected sensors
2. Initializes sensors and CAN interface
3. Begins transmitting sensor data
4. Responds to control commands via CAN

Monitor CAN messages:
```bash
candump can0
```

### CAN Sensor Tool

The `can-sensor-tool` is the primary management interface for ESP32 sensor nodes.

#### Device Discovery
```bash
# Ping all nodes (discover devices on the bus)
can-sensor-tool ping

# Get device info for node 0
can-sensor-tool info

# Get device info for specific node
can-sensor-tool --node=1 info
```

#### Monitoring
```bash
# Monitor all sensor messages
can-sensor-tool monitor

# Monitor specific node
can-sensor-tool --node=1 monitor
```

#### Node Management
```bash
# Stop sensor transmission
can-sensor-tool stop

# Start sensor transmission
can-sensor-tool start

# Reboot node (saves calibration first)
can-sensor-tool reboot

# Factory reset (clears calibration)
can-sensor-tool factory-reset

# Identify node (blink onboard LED for 5 seconds)
can-sensor-tool identify

# Change node ID (0-5)
can-sensor-tool set-id 2

# Commands for specific node
can-sensor-tool --node=1 reboot
```

#### A/B Firmware Updates
```bash
# Upload new firmware to node 0
can-sensor-tool update ./build/esp32-can-sensor.bin

# Upload to specific node
can-sensor-tool --node=1 update ./build/esp32-can-sensor.bin

# Upload with custom chunk delay (for noisy CAN buses)
can-sensor-tool --chunk-delay=5 update ./build/esp32-can-sensor.bin
```

**OTA Features:**
- A/B partition scheme with automatic rollback
- ~3 KB/s transfer speed over CAN
- Progress display with ETA
- Automatic retry with exponential backoff
- Safe: keeps old firmware until new one is validated

### A/B OTA Update Mechanism

The firmware uses ESP-IDF's standard A/B OTA partition scheme for safe, reliable updates:

```
Flash Layout (4MB):
┌─────────────────┐ 0x000000
│   Bootloader    │
├─────────────────┤ 0x009000
│      NVS        │ (24KB - calibration, node ID)
├─────────────────┤ 0x010000
│    OTA Data     │ (8KB - tracks active partition)
├─────────────────┤ 0x020000
│     OTA_0       │ (~1.94MB - firmware slot A)
├─────────────────┤ 0x210000
│     OTA_1       │ (~1.94MB - firmware slot B)
└─────────────────┘ 0x400000
```

**How it works:**
1. Initial USB flash writes firmware to OTA_0
2. First CAN OTA update writes to OTA_1, reboots to OTA_1
3. Next CAN OTA update writes to OTA_0, reboots to OTA_0
4. Continues alternating (ping-pong) between slots

**Automatic Rollback:**
- New firmware must call `esp_ota_mark_app_valid()` within first boot
- If firmware crashes before validation, bootloader reverts to previous slot
- After 3 failed boot attempts, automatically rolls back

**Example OTA session:**
```bash
# Check current partition
$ can-sensor-tool info
Device Info (Node 0):
  Firmware:    v1.0.1
  Partition:   ota_0 (valid)     # Currently running from OTA_0

# Perform OTA update
$ can-sensor-tool update ./build/esp32-can-sensor.bin
Uploading: [==============================] 100% (345040/345040) 3.3 KB/s
Update complete! Node 0 is rebooting to new firmware.

# Verify - now on OTA_1
$ can-sensor-tool info
Device Info (Node 0):
  Firmware:    v1.0.1
  Partition:   ota_1 (valid)     # Now running from OTA_1
```

### Direct CAN Commands

For basic control without can-sensor-tool:
```bash
# Start/Stop transmission (Node 0)
cansend can0 111#        # Start
cansend can0 110#        # Stop

# Graceful shutdown (saves state)
cansend can0 112#

# Reboot (saves state and reboots)
cansend can0 113#

# Factory reset
cansend can0 114#

# Identify (blink LED for 5 seconds)
cansend can0 11A#
```

## CAN Protocol

### Node Addressing

Each node has 32 message IDs reserved (0x20 spacing), supporting up to 16 nodes:

| Node ID | Base Address | Sensor Data | Control Cmds | OTA Commands |
|---------|--------------|-------------|--------------|--------------|
| 0 | 0x100 | 0x100-0x10F | 0x110-0x11F | 0x700/0x708 |
| 1 | 0x120 | 0x120-0x12F | 0x130-0x13F | 0x710/0x718 |
| 2 | 0x140 | 0x140-0x14F | 0x150-0x15F | 0x720/0x728 |
| ... | ... | ... | ... | ... |
| 15 | 0x2E0 | 0x2E0-0x2EF | 0x2F0-0x2FF | 0x7F0/0x7F8 |

### Message Types (Node 0 Example)

**Sensor Data (offsets 0x00-0x0F):**

| ID | Direction | Purpose | Rate |
|----|-----------|---------|------|
| 0x100 | ESP32 → PC | Ambient light data | 1 Hz |
| 0x101 | ESP32 → PC | Environmental (T/H/P) | 0.33 Hz |
| 0x102 | ESP32 → PC | Air quality (IAQ/CO2/VOC) | 0.33 Hz |
| 0x103-0x106 | ESP32 → PC | BME688 gas selectivity (classes 1-4) | Future |
| 0x107 | ESP32 → PC | mm-wave presence detection | Future |
| 0x108 | ESP32 → PC | Presence extended data | Future |
| 0x10F | ESP32 → PC | System status | 0.1 Hz |

**Control Commands (offsets 0x10-0x1F):**

| ID | Direction | Purpose |
|----|-----------|---------|
| 0x110 | PC → ESP32 | Stop transmission |
| 0x111 | PC → ESP32 | Start transmission |
| 0x112 | PC → ESP32 | Graceful shutdown |
| 0x113 | PC → ESP32 | Reboot |
| 0x114 | PC → ESP32 | Factory reset |
| 0x115 | PC → ESP32 | Set node ID |
| 0x116 | PC → ESP32 | Get device info |
| 0x117 | ESP32 → PC | Device info response |
| 0x118 | PC → ESP32 | Discovery ping |
| 0x119 | ESP32 → PC | Discovery pong |

### Message Formats

#### Ambient Light (offset 0x00)
```
Byte 0-2: Lux value (24-bit LE, 0-16.7M lux)
Byte 3:   Status (0x00=OK, 0x01=Error)
Byte 4:   Sequence counter (0-255)
Byte 5:   Config index (0-20=VEML7700, 100-111=OPT4001, 200-211=OPT3001)
Byte 6-7: Checksum (16-bit LE)
```

#### Environmental (offset 0x01)
```
Byte 0-1: Temperature (int16_t, 0.01°C)
Byte 2:   Humidity (%RH)
Byte 3-4: Pressure (hPa × 10)
Byte 5:   Status
Byte 6-7: Checksum
```

#### Air Quality (offset 0x02)
```
Byte 0-1: IAQ index (0-500)
Byte 2:   IAQ accuracy (0-3)
Byte 3-4: CO2 equivalent (ppm)
Byte 5-6: Breath VOC (ppm)
Byte 7:   Status
```

#### Device Info Response (offset 0x17)
```
Byte 0:   Node ID
Byte 1:   Firmware major version
Byte 2:   Firmware minor version
Byte 3:   Firmware patch version
Byte 4:   Sensor flags (bit 0: ALS, bit 1: BME680, etc.)
Byte 5:   ALS type (0=none, 1=VEML7700, 2=OPT4001, 3=OPT3001)
Byte 6:   Status flags (bit 0: transmitting)
Byte 7:   Partition info (bits 0-2: type, bits 4-6: OTA state)
```

## File Structure

```
Esp32-CAN-ALS/
├── main/
│   ├── main.c              # Task coordination
│   ├── als_driver.c/h      # Ambient light abstraction
│   ├── veml7700_driver.c/h # VEML7700 driver
│   ├── opt4001_driver.c/h  # OPT4001 driver
│   ├── bme680_bsec.c/h     # BME680/688 with BSEC
│   ├── can_protocol.c/h    # CAN message formatting
│   ├── ota_handler.c/h     # A/B OTA updates
│   └── sensor_common.h     # Shared data structures
├── tools/
│   ├── can-sensor-tool.cpp # Node management tool
│   ├── CAN_SENSOR_TOOL.md  # Tool documentation
│   └── Makefile
├── components/
│   └── bsec/               # Bosch BSEC library
├── CLAUDE.md               # Development guidelines
└── README.md               # This file
```

## Technical Specifications

### Sensor Specifications

| Sensor | Measurement | Range | Accuracy |
|--------|-------------|-------|----------|
| VEML7700 | Lux | 0-120K | ±1.4% @ 70K |
| OPT3001 | Lux | 0-83K | ±2% @ 100 |
| OPT4001 | Lux | 0-2.2M | ±1.5% @ 870 |
| BME680/688 | Temperature | -40 to +85°C | ±1.0°C |
| BME680/688 | Humidity | 0-100% | ±3% RH |
| BME680/688 | Pressure | 300-1100 hPa | ±1.0 hPa |
| BME680/688 | IAQ | 0-500 | BSEC calibrated |

### System Specifications

- **CAN Bus Speed:** 500 kbps
- **I2C Speed:** 100 kHz
- **OTA Transfer Speed:** ~3.3 KB/s
- **Firmware Size:** ~302-345 KB (15-17% of partition)
- **OTA Partition Size:** ~1.94 MB each (A/B slots)
- **RAM Usage:** ~90 KB
- **Max Nodes:** 16 (IDs 0-15)

## Troubleshooting

### Common Issues

**CAN messages not visible:**
- Verify CAN interface is up: `ip link show can0`
- Check TJA1050 has 5V supply
- Verify termination resistors (120Ω at both ends)

**Sensor not detected:**
- Check I2C connections (SDA/SCL)
- Verify correct I2C address for sensor
- Use `can-sensor-tool info` to see detected sensors

**OTA update fails:**
- Ensure node is responding: `can-sensor-tool ping`
- Try with longer chunk delay: `can-sensor-tool --chunk-delay=10 update firmware.bin`
- Check for CAN bus errors: `ip -s link show can0`

**IAQ readings stuck at defaults:**
- BME680 needs calibration time (5+ minutes for accuracy > 0)
- Use graceful shutdown to save calibration state

### Debug Commands
```bash
# Monitor all CAN traffic with timestamps
candump can0 -t z

# Check CAN interface statistics
ip -s -d link show can0

# Get device info
can-sensor-tool info

# Check all nodes
can-sensor-tool ping
```

## License

This project is open source (GPLv3). Feel free to modify and distribute according to your needs.

## BSEC Licensing Note

The BME680/BME688 air-quality features use Bosch Sensortec’s **BSEC 2.x** library, which is proprietary and not bundled in this repo. To enable it, download BSEC from Bosch, accept the license, and point `build.sh` to the zip (see `BME680_SETUP.md`).

## Contributing

Contributions welcome for:
- Additional sensor support
- Protocol enhancements
- Documentation improvements
- Testing and bug fixes

See [CLAUDE.md](CLAUDE.md) for detailed development guidelines.
