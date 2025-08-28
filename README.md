# CAN based Ambient-Light-Sensor using ESP32

A professional-grade automotive ambient light sensor implementation using ESP32 microcontroller with CAN bus communication and calibration capabilities.

## Overview

This project implements an ambient light sensor node for automotive CAN networks using:
- **ESP32 microcontroller** (C6, C3, or other variants with CAN support)
- **VEML7700 ambient light sensor** (I2C interface)
- **TJA1050 CAN transceiver** for robust automotive communication
- **Calibration system** with reference lux meter support
- **Structured CAN messaging** with error detection and status reporting

## Features

- Real-time ambient light measurement (0-65535 lux range)
- Automotive CAN bus integration (500 kbps)
- Professional calibration system with persistent storage
- Structured 8-byte CAN message format with checksums
- START/STOP remote control capability
- Raw sensor data access during calibration
- Cross-platform calibration utility (C++)

## Hardware Requirements

### Core Components
- ESP32 development board (ESP32-C6, ESP32-C3, or compatible)
- VEML7700 ambient light sensor breakout board
- TJA1050 CAN transceiver breakout board
- CAN-to-USB adapter (for development/calibration)

### TJA1050 CAN Transceiver Board
Standard breakout board with:
- **CAN side**: CANH, CANL pins
- **MCU side**: VCC, GND, TX, RX pins
- **Power**: 5V supply required for proper CAN signal levels
- **Logic**: 3.3V compatible TX/RX pins

## Wiring Diagram

```
ESP32               VEML7700            TJA1050             CAN Bus
-----               --------            -------             -------
GPIO6 (SDA)    <--> SDA
GPIO7 (SCL)    <--> SCL
3.3V           ---> VCC
GND            ---> GND

GPIO4 (CAN_TX) ---> TX
GPIO5 (CAN_RX) <--- RX
5V             ---> VCC
GND            ---> GND                 GND <-----------> GND
                                       CANH -----------> CAN_HIGH
                                       CANL -----------> CAN_LOW
```

**Important**: TJA1050 requires 5V supply for proper differential signal levels, even though logic pins are 3.3V compatible.

## Software Prerequisites

### ESP-IDF Installation
```bash
# Install dependencies (Ubuntu/Debian)
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dkms libusb-1.0-0

# Clone ESP-IDF
mkdir ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git

# Install ESP-IDF
cd ~/esp/esp-idf
./install.sh esp32c6

# Setup environment (add to ~/.bashrc)
. ~/esp/esp-idf/export.sh
```

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
```bash
cd Esp32-CAN-ALS/

# Set target (adjust for your ESP32 variant)
idf.py set-target esp32c6

# Configure (optional)
idf.py menuconfig

# Build and flash
idf.py build flash monitor
```

### Calibration Utility
```bash
cd calibrate/
make

# Make executable available system-wide (optional)
sudo make install
```

## Usage

### Basic Operation

Once flashed, the ESP32 automatically:
1. Initializes VEML7700 sensor and CAN interface
2. Begins transmitting lux readings every second
3. Responds to START/STOP commands via CAN

Monitor CAN messages:
```bash
candump can0
```

Expected output:
```
can0  0A1   [8]  00 00 00 00 00 00 00 00    # Start command
can0  0A2   [8]  1A 00 00 01 00 00 1B 00    # Lux data (26 lux, seq=1)
can0  0A2   [8]  1F 00 00 02 00 00 21 00    # Lux data (31 lux, seq=2)
```

### Remote Control
```bash
# Stop transmission
cansend can0 0A0#

# Start transmission  
cansend can0 0A1#
```

### Sensor Calibration

#### View Current Readings
```bash
./calibrate_lux_sensor --show-current
```

#### Perform Calibration
1. Place reference lux meter next to VEML7700 sensor
2. Read reference value (e.g., 150 lux)
3. Run calibration:
```bash
./calibrate_lux_sensor --reference=150
```

#### Verify Current Offset
```bash
./calibrate_lux_sensor --get-offset
```

The calibration offset is permanently stored in ESP32 NVS (non-volatile storage) and survives power cycles.

## CAN Message Format

### Message IDs
| ID    | Direction | Purpose |
|-------|-----------|---------|
| 0x0A0 | PC → ESP32 | Stop transmission |
| 0x0A1 | PC → ESP32 | Start transmission |  
| 0x0A2 | ESP32 → PC | Lux data (normal operation) |
| 0x0A3 | PC → ESP32 | Calibration commands |
| 0x0A4 | ESP32 → PC | Calibration responses |
| 0x0A5 | ESP32 → PC | Raw sensor data (calibration mode) |

### Lux Data Message (0x0A2)
8-byte structured format:
```
Byte 0-1: Lux value (16-bit, little-endian)
Byte 2:   Sensor status (0x00=OK, 0x01=Error, 0x02=Calibrating)  
Byte 3:   Sequence counter (0-255, rolling)
Byte 4-5: Reserved (0x00)
Byte 6-7: Checksum (16-bit sum of bytes 0-5, little-endian)
```

#### Example Message
```
0A2 [8] 1A 00 00 05 00 00 1F 00
        ^^^^^ ^^ ^^ ^^^^^ ^^^^^
        │     │   │   │     └─ Checksum (0x001F = 31)
        │     │   │   └─ Reserved  
        │     │   └─ Auto-Increasing Sequence counter (5)
        │     └─ Status (0=OK)
        └─ Lux value (0x001A = 26 lux)
```

### Calibration Commands (0x0A3)
| Command | Value | Data | Purpose |
|---------|-------|------|---------|
| Enter   | 0x01  | -    | Enter calibration mode |
| Set Ref | 0x02  | 16-bit lux | Set reference value |
| Save    | 0x03  | -    | Save calibration to NVS |
| Exit    | 0x04  | -    | Exit calibration mode |
| Get     | 0x05  | -    | Get current offset |

### Calibration Responses (0x0A4)
```
Byte 0: Command echo
Byte 1: Status (0x00=OK, 0x01=Error)
Byte 2-7: Response data (command-specific)
```

### Raw Sensor Data (0x0A5)
During calibration mode only:
```
Byte 0-1: Raw lux value (16-bit, little-endian)
Byte 2-3: Marker bytes (0xAA, 0x55)
Byte 4-7: Reserved (0x00)
```

## Troubleshooting

### Common Issues

**CAN messages not visible:**
- Verify CAN interface is up: `ip link show can0`
- Check wiring connections, especially ground
- Confirm 5V supply to TJA1050

**Sensor initialization fails:**
- Check I2C connections (SDA/SCL)
- Try external 4.7kΩ pull-up resistors on I2C lines
- Verify VEML7700 address (0x10)

**Calibration not working:**
- Ensure ESP32 is running (check CAN messages)
- Verify calibration utility can bind to CAN interface
- Check for proper response messages (0x0A4)

**Serial monitor crashes:**
- Known ESP-IDF 5.5 + ESP32-C6 issue with USB-Serial/JTAG
- Use CAN monitoring instead: `candump can0`
- Monitor does not affect core functionality

### Debug Commands
```bash
# Monitor all CAN traffic
candump can0 -t z

# Check CAN interface status  
ip -s -d link show can0

# Test basic CAN communication
cansend can0 0A0#  # Should stop ESP32 transmission
```

## File Structure
```
Esp32-CAN-ALS/
├── main/
│   ├── main.c              # ESP32 firmware
│   └── CMakeLists.txt      # Build configuration
├── calibration/
│   ├── calibrate_lux_sensor.cpp  # Calibration utility
│   ├── Makefile            # Build configuration
├── CMakeLists.txt          # Project configuration
└── README.md               # This file
```

## Technical Specifications

- **Measurement Range**: 0-65535 lux (limited by 16-bit CAN message format)
- **Sensor Resolution**: 0.0036 lux/count (VEML7700 default configuration)
- **Update Rate**: 1 Hz (1 second intervals)
- **CAN Bus Speed**: 500 kbps
- **I2C Speed**: 100 kHz
- **Calibration Storage**: NVS (non-volatile, survives power cycles)
- **Message Validation**: 16-bit checksum per message

## License

This project is open source(GPLv3). Feel free to modify and distribute according to your needs.

## Contributing

Contributions welcome for:
- Additional sensor support
- Enhanced calibration algorithms  
- Alternative CAN transceivers
- Documentation improvements
