# CAN based Ambient-Light-Sensor using ESP32

A professional-grade automotive ambient light sensor implementation using ESP32 microcontroller with CAN bus communication and runtime sensor auto-detection.

## Overview

This project implements an ambient light sensor node for automotive CAN networks using:
- **ESP32 microcontroller** (C6, C3, or other variants with CAN support)
- **Dual sensor support** with runtime auto-detection:
  - **VEML7700** (0-120K lux, calibrated, I2C 0x10)
  - **OPT4001** (0-2.2M lux, factory calibrated, I2C 0x44)
- **TJA1050 CAN transceiver** for robust automotive communication
- **Modular architecture** with queue-based data flow
- **Structured CAN messaging** with error detection and status reporting

## Features

- **Dual sensor support** with runtime auto-detection
  - VEML7700: 0-120K lux with piecewise calibration
  - OPT4001: 0-2.2M lux, factory calibrated, ±1.5% accuracy
- **Wide measurement range**: 0-16.7M lux (24-bit CAN message format)
- **Automotive CAN bus** integration (500 kbps)
- **Intelligent auto-ranging**:
  - VEML7700: 21 software configurations
  - OPT4001: 12 hardware ranges
- **Structured 8-byte CAN message** format with checksums
- **START/STOP remote control** capability
- **Sensor identification** via config index in CAN messages

## Hardware Requirements

### Core Components
- ESP32 development board (ESP32-C6, ESP32-C3, or compatible)
- **Choose ONE ambient light sensor:**
  - VEML7700 ambient light sensor breakout board (I2C address 0x10), OR
  - OPT4001 ambient light sensor breakout board (I2C address 0x44)
- TJA1050 CAN transceiver breakout board
- CAN-to-USB adapter (for development/testing)

**Note:** The firmware automatically detects which sensor is connected at runtime.

### TJA1050 CAN Transceiver Board
Standard breakout board with:
- **CAN side**: CANH, CANL pins
- **MCU side**: VCC, GND, TX, RX pins
- **Power**: 5V supply required for proper CAN signal levels
- **Logic**: 3.3V compatible TX/RX pins

## Wiring Diagram

```
ESP32          VEML7700/OPT4001         TJA1050             CAN Bus
-----          ----------------         -------             -------
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

**Important Notes:**
- TJA1050 requires 5V supply for proper differential signal levels (logic pins are 3.3V compatible)
- Only connect ONE sensor at a time (either VEML7700 or OPT4001)
- Both sensors use same I2C pins but different addresses (auto-detected)

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
git clone https://github.com/hackboxguy/Esp32-CAN-ALS.git
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
1. Auto-detects connected sensor (VEML7700 or OPT4001)
2. Initializes sensor and CAN interface
3. Begins transmitting lux readings every second
4. Responds to START/STOP commands via CAN

Monitor CAN messages:
```bash
candump can0
```

Expected output:
```
can0  0A1   [8]  00 00 00 00 00 00 00 00    # Start command
can0  0A2   [8]  1A 00 00 01 00 08 23 00    # Lux data (26 lux, config=8)
can0  0A2   [8]  1F 00 00 02 00 08 29 00    # Lux data (31 lux, config=8)
```

**Identifying Connected Sensor:**
- Byte 5 (config index):
  - 0-20 = VEML7700 detected
  - 100-111 = OPT4001 detected

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
Byte 0-2: Lux value (24-bit, little-endian, 0-16.7M lux max)
Byte 3:   Sensor status (0x00=OK, 0x01=Error)
Byte 4:   Sequence counter (0-255, rolling)
Byte 5:   Config index (0-20=VEML7700, 100-111=OPT4001)
Byte 6-7: Checksum (16-bit sum of bytes 0-5, little-endian)
```

#### Example Message (VEML7700)
```
0A2 [8] 1A 00 00 00 05 08 27 00
        ^^^^^^^^ ^^ ^^ ^^ ^^^^^
        │        │  │  │  └─ Checksum (0x0027)
        │        │  │  └─ Config index (8 = VEML7700)
        │        │  └─ Sequence counter (5)
        │        └─ Status (0=OK)
        └─ Lux value (0x00001A = 26 lux)
```

#### Example Message (OPT4001)
```
0A2 [8] 7A 03 00 00 0A 66 ED 00
        ^^^^^^^^ ^^ ^^ ^^ ^^^^^
        │        │  │  │  └─ Checksum (0x00ED)
        │        │  │  └─ Config index (102 = OPT4001, range 2)
        │        │  └─ Sequence counter (10)
        │        └─ Status (0=OK)
        └─ Lux value (0x00037A = 890 lux)
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
- Verify sensor address:
  - VEML7700: 0x10
  - OPT4001: 0x44
- Check serial monitor for "Auto-detecting ambient light sensor..." message
- Only one sensor should be connected at a time

**Wrong sensor detected:**
- Verify I2C address (some breakout boards have configurable addresses)
- Check config index in CAN messages:
  - 0-20 = VEML7700
  - 100-111 = OPT4001

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

### Sensor Comparison
| Feature | VEML7700 | OPT4001 |
|---------|----------|---------|
| Range | 0-120K lux | 0-2.2M lux |
| Auto-ranging | 21 SW configs | 12 HW ranges |
| Calibration | Piecewise linear | Factory (SOT-5x3) |
| Accuracy | ±1.4% @ 70K lux | ±1.5% @ 870 lux |
| I2C Address | 0x10 | 0x44 |
| Config Index | 0-20 | 100-111 |

### System Specifications
- **Measurement Range**: 0-16,777,215 lux (24-bit CAN message format)
- **Update Rate**: 1 Hz (1 second intervals)
- **CAN Bus Speed**: 500 kbps
- **I2C Speed**: 100 kHz
- **Message Validation**: 16-bit checksum per message
- **Flash Usage**: ~220 KB (dual sensor support)
- **RAM Usage**: ~75 KB

## License

This project is open source(GPLv3). Feel free to modify and distribute according to your needs.

## Contributing

Contributions welcome for:
- Additional sensor support
- Enhanced calibration algorithms  
- Alternative CAN transceivers
- Documentation improvements
