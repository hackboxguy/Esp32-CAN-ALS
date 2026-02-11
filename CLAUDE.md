# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Summary

ESP32-C3/C6 automotive CAN node with **multi-sensor support**: ambient light sensors (VEML7700/OPT3001/OPT4001) and environmental sensors (BME680/BME688). Production-ready implementation with runtime auto-detection, intelligent auto-ranging, A/B OTA firmware updates over CAN, and modular queue-based architecture designed for expansion.

**Sensor Support:**
- **Ambient Light:** VEML7700 (0-120K lux, calibrated) OR OPT3001 (0-83K lux) OR OPT4001 (0-2.2M lux, factory calibrated)
- **Environmental:** BME680/BME688 with BSEC 2.x (temperature, humidity, pressure, air quality)

**Key Capabilities:**
- A/B OTA firmware updates over CAN bus (~3 KB/s) with automatic rollback
- Multi-node support (up to 16 nodes, configurable IDs 0-15)
- `can-sensor-tool` CLI for node discovery, monitoring, OTA updates, and management

## Build and Development Commands

### ESP32 Firmware (using build.sh - preferred)

```bash
# First build (specify target)
./build.sh --target=esp32c6                    # or esp32c3
./build.sh --target=esp32c6 --version=1.2.3    # with firmware version

# Rebuild (target remembered from previous build)
./build.sh

# Build and flash
./build.sh --flash --port=/dev/ttyUSB0

# Flash existing binary without rebuilding
./build.sh --flash-only --port=/dev/ttyUSB0

# Clean build
./build.sh --clean --target=esp32c6

# Build with BSEC library for BME680/688 support
./build.sh --target=esp32c6 --bsecpath=/path/to/bsec2-6-1-0_generic_release_22102024.zip
```

### ESP32 Firmware (using idf.py directly)

```bash
# Set target for your ESP32 variant
idf.py set-target esp32c6    # or esp32c3

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor

# Clean build
idf.py fullclean && idf.py build

# Check binary size
idf.py size-components
```

### CAN Sensor Tool (C++ node management CLI)

```bash
cd tools/
make                    # Build
sudo make install       # Install to /usr/local/bin

# Usage
can-sensor-tool ping                           # Discover all nodes on CAN bus
can-sensor-tool info                           # Get device info for node 0
can-sensor-tool --node=1 info                  # Get device info for node 1
can-sensor-tool monitor                        # Monitor sensor data (dashboard)
can-sensor-tool monitor --verbose              # Monitor with scrolling output
can-sensor-tool start                          # Start sensor transmission
can-sensor-tool stop                           # Stop sensor transmission
can-sensor-tool reboot                         # Reboot node (saves calibration)
can-sensor-tool factory-reset                  # Factory reset (clears calibration)
can-sensor-tool set-id 2                       # Change node ID
can-sensor-tool identify                       # Blink onboard LED for 5 seconds
can-sensor-tool --node=2 identify              # Identify specific node
can-sensor-tool update ./build/esp32-can-sensor.bin  # OTA firmware update

# Multi-node commands (--all or --nodes=)
can-sensor-tool --all monitor                  # Dashboard with all nodes
can-sensor-tool --all info                     # Show info for all nodes
can-sensor-tool --all identify                 # Blink LED on all nodes
can-sensor-tool --nodes=0,2 reboot             # Reboot specific nodes
can-sensor-tool --all update firmware.bin      # OTA update all nodes (with partition verification)
```

### Calibration Utility (C++)

```bash
cd calibration/
make                    # Build
sudo make install       # Install to /usr/local/bin

# Usage
./calibrate_lux_sensor --show-current           # View current readings
./calibrate_lux_sensor --reference=150          # Calibrate with reference
./calibrate_lux_sensor --get-offset             # View stored calibration
./calibrate_lux_sensor --show-calibrated        # Show calibrated output
./calibrate_lux_sensor --reset-calibration      # Reset to factory defaults
```

### CAN Bus Setup (Linux)

```bash
# Setup CAN interface (500 kbps)
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Monitor messages
candump can0 -L

# Control commands (Node 0)
cansend can0 111#        # Start transmission
cansend can0 110#        # Stop transmission
cansend can0 112#        # Graceful shutdown (save state)
cansend can0 113#        # Reboot ESP32 (save state first)
cansend can0 114#        # Factory reset (clear calibration)
cansend can0 11A#        # Identify (blink LED for 5 seconds)
```

### Testing

```bash
# Run firmware and monitor serial output
idf.py monitor

# In separate terminal, monitor CAN bus
candump can0 -t z

# Test basic functionality
cansend can0 111#        # Start transmission (Node 0)
# Should see:
#   0x100 @ 1 Hz (ambient light - VEML7700 or OPT4001)
#   0x101 @ 0.33 Hz (BME680 environmental - T/H/P)
#   0x102 @ 0.33 Hz (BME680 air quality - IAQ/CO2/VOC)

# Test graceful shutdown
cansend can0 112#        # Shutdown (saves BSEC state)
# Wait for "BSEC state saved to NVS" message

# Test reboot with state persistence
cansend can0 113#        # Reboot (saves and reboots)
# Watch for IAQ accuracy to persist after reboot

# Test factory reset
cansend can0 114#        # Factory reset
# IAQ accuracy should return to 0 after reboot
```

## Code Architecture

### High-Level Structure (Modular)

```
main/
├── main.c (~695 lines) - Task coordination and initialization
│   ├── sensor_poll_task: Periodic sensor reading (1 Hz)
│   ├── bme680_sensor_task: BSEC-driven environmental reading (3s)
│   ├── twai_receive_task: CAN command reception + OTA dispatch
│   ├── twai_transmit_task: CAN message transmission from queue
│   └── twai_control_task: Auto-start trigger (one-shot)
│
├── sensor_common.h (~65 lines) - Shared data structures
│   ├── sensor_id_t: Sensor enumeration (VEML7700, BME680, LD2410, MQ3)
│   ├── sensor_data_t: Unified sensor data union
│   └── Status codes and constants
│
├── firmware_config.h (~26 lines) - Feature flags and version
│   └── Version overridable via CMake -D flags
│
├── als_driver.c/h (~300 lines) - Ambient Light Sensor abstraction layer
│   ├── Runtime auto-detection (I2C address probing: 0x10, 0x44)
│   ├── Unified API for VEML7700, OPT3001, or OPT4001
│   ├── I2C bus initialization
│   └── Sensor type identification via config index
│
├── veml7700_driver.c/h (~540 lines) - VEML7700 sensor module
│   ├── I2C communication functions
│   ├── Configuration table (21 configs: gain × integration time)
│   ├── Intelligent auto-ranging algorithm with saturation ramp-up
│   ├── Moving average filter (8 samples)
│   └── Calibration correction (piecewise linear, 0-120K lux range)
│
├── opt4001_driver.c/h (~270 lines) - OPT4001 sensor module
│   ├── I2C communication (big-endian)
│   ├── Hardware auto-ranging (12 ranges, 0-11)
│   ├── 800ms integration time (highest accuracy)
│   ├── Factory-calibrated (SOT-5x3 package)
│   └── Config index: 100-111
│
├── opt3001_driver.c/h (~280 lines) - OPT3001 sensor module
│   ├── I2C communication (I2C address 0x44, shared with OPT4001)
│   ├── Hardware auto-ranging (12 ranges, 0-11)
│   ├── 800ms conversion time, continuous mode
│   ├── Factory-calibrated (0-83,865 lux range)
│   └── Config index: 200-211
│
├── bme680_bsec.c/h (~1075 lines) - BME680/BME688 environmental sensor with BSEC
│   ├── Runtime auto-detection (I2C address 0x76/0x77)
│   ├── BSEC 2.6.1.0 library integration (IAQ/CO2/VOC enabled)
│   ├── Virtual sensor subscription with 8 BSEC outputs
│   ├── Fixed 3-second reading interval (LP mode)
│   ├── Gas heater enabled (380°C, 1000ms for IAQ measurement)
│   ├── NVS state management (auto-saves every 4 hours)
│   └── Calibration persistence across reboots
│
├── ota_handler.c/h (~575 lines) - A/B OTA firmware updates over CAN
│   ├── State machine: IDLE → RECEIVING → VERIFYING → COMPLETE
│   ├── 6-byte chunks per CAN frame with sequence numbers
│   ├── CRC32 verification of complete firmware image
│   ├── Automatic rollback on failed boot (3 attempts)
│   └── Session timeout (5 min) and retry logic (3 retries)
│
├── can_protocol.c/h (~535 lines) - CAN message formatting & protocol definitions
│   ├── Node addressing: base 0x100, spacing 0x20 (32 IDs/node)
│   ├── Message ID definitions for all sensors and control commands
│   ├── OTA protocol message IDs (0x700 base)
│   ├── Format functions for lux, environmental, air quality messages
│   └── Checksum calculation
│
tools/
├── can-sensor-tool.cpp (~81K) - Complete C++ CLI for node management
│   ├── Device discovery (ping all nodes)
│   ├── Sensor monitoring with decoded output
│   ├── Node control (start/stop/reboot/factory-reset)
│   ├── Node ID configuration
│   └── OTA firmware updates with progress display
│
calibration/
└── calibrate_lux_sensor.cpp (~29K) - Interactive lux calibration utility
```

### Data Flow Architecture

```
┌─────────────────┐
│ sensor_poll_task│──┐
│   (ALS, 1 Hz)   │  │
│   (Priority 5)  │  │
└─────────────────┘  │
                     │    ┌──────────────┐    ┌──────────────────┐
┌─────────────────┐  ├───→│ sensor_queue │───→│ twai_transmit    │──→ CAN Bus
│ bme680_sensor   │  │    │  (depth: 20) │    │    _task (P8)    │
│   _task (3s)    │──┘    └──────────────┘    └──────────────────┘
│   (Priority 5)  │                                    ▲
└─────────────────┘                                    │
                                                       │
                                             ┌─────────┴────────┐
                                             │ twai_receive_task│
                                             │   (Priority 9)   │
                                             │ START/STOP cmds  │
                                             └──────────────────┘
```

**Queue-Based Design:**
- Sensor tasks read hardware and push to shared queue
- Transmit task consumes from queue and sends via CAN
- Decouples sensor reading from CAN transmission
- Allows multiple sensors without blocking

### Key Algorithms

**Auto-Ranging Logic** [veml7700_driver.c:327-410]
- Maintains reading history to prevent oscillation
- Single-step configuration changes with intelligent saturation handling
- **Saturation ramp-up:** Aggressively steps through configs when saturated (>65K counts)
- Skips settling period during saturation ramp to quickly find optimal range
- Targets optimal ADC range: 10,000-40,000 counts (out of 65,535)
- Configuration table ordered by sensitivity (most sensitive → least sensitive)
- **21 configurations** spanning 0-120,000 lux dynamic range:
  - Configs 0-4: 2x gain (highest sensitivity, <500 lux)
  - Configs 5-10: 1x gain (500-15K lux)
  - Configs 11-14: 1/4x gain (5K-30K lux)
  - Configs 15-20: 1/8x gain (10K-120K lux, direct sunlight)

**Moving Average Filter** [veml7700_driver.c:167-179]
- Circular buffer: 8 samples
- Updates on every reading
- Reduces high-frequency noise while maintaining responsiveness

**Calibration Correction** [veml7700_driver.c:146-164]
- Piecewise linear function compensating for:
  - Adafruit VEML7700 board diffuser dome optical properties
  - Non-linear sensor response at different light intensities
- **Calibration ranges:**
  - <10 lux: 0.85x (very dim conditions)
  - 10-100 lux: 0.92x (dim indoor)
  - 100-500 lux: 1.05x (normal indoor)
  - 500-1K lux: 1.12x (bright indoor)
  - 1K-10K lux: 1.18x (very bright indoor / overcast outdoor)
  - **>10K lux: 0.915x (direct sunlight - diffuser non-linearity)**
- Calibrated against reference lux meter at 70K lux (±1.4% accuracy)
- Persistent storage via NVS (future enhancement)

### Ambient Light Sensor Support

The firmware supports three ambient light sensors with **runtime auto-detection**:

**VEML7700** (I2C address 0x10):
- **Range:** 0-120,000 lux
- **Auto-ranging:** 21 software configurations (gain × integration time)
- **Calibration:** Piecewise linear correction for diffuser dome
- **Accuracy:** ±1.4% @ 70K lux (vs reference meter)
- **Best for:** Applications requiring calibrated measurements across full range
- **Config index:** 0-20 (reported in CAN messages)

**OPT4001** (I2C address 0x44):
- **Range:** 0-2,200,000 lux (2.2M lux)
- **Auto-ranging:** 12 hardware ranges (managed by sensor)
- **Calibration:** Factory calibrated (SOT-5x3 package: 0.0004375 LSB)
- **Accuracy:** ±1.5% @ 870 lux (vs reference meter)
- **Integration time:** 800ms (highest accuracy)
- **Best for:** Wide dynamic range applications, direct sunlight measurement
- **Config index:** 100-111 (range 0-11 + 100 offset)

**OPT3001** (I2C address 0x44):
- **Range:** 0-83,865 lux
- **Auto-ranging:** 12 hardware ranges (managed by sensor)
- **Calibration:** Factory calibrated
- **Formula:** `lux = 0.01 × 2^exponent × mantissa`
- **Integration time:** 800ms (continuous conversion mode)
- **Best for:** Mid-range applications, cost-effective alternative
- **Config index:** 200-211 (range 0-11 + 200 offset)
- **Note:** Shares I2C address 0x44 with OPT4001; only one can be connected

**Auto-Detection Logic** [als_driver.c]:
1. Initialize I2C bus
2. Probe address 0x10 (VEML7700)
   - If found → initialize VEML7700 driver
3. Probe address 0x44 (OPT4001 / OPT3001)
   - Device ID distinguishes OPT4001 vs OPT3001
   - If found → initialize appropriate driver
4. Use detected sensor transparently via unified API

**Sensor Identification:**
- CAN clients can identify sensor type via config index:
  - Config 0-20 = VEML7700
  - Config 100-111 = OPT4001
  - Config 200-211 = OPT3001

### BME680/BME688 Environmental Sensor Support

The firmware supports BME680/BME688 environmental sensors with **runtime auto-detection** and **full BSEC 2.6.1.0 integration** for air quality monitoring.

**Current Implementation (Full IAQ Mode):**
- ✅ **BSEC Virtual Sensor Subscription** - 8 outputs configured
- ✅ **IAQ (Indoor Air Quality)** - Index 0-500 with accuracy tracking (0-3)
- ✅ **CO2 Equivalent** - Estimated CO2 in ppm
- ✅ **Breath VOC Equivalent** - Volatile organic compounds in ppm
- ✅ **Temperature/Humidity/Pressure** - Heat-compensated and raw values
- ✅ **Gas Heater** - 380°C for 1000ms (BSEC-controlled)
- ✅ **Calibration Persistence** - NVS storage with 4-hour auto-save
- ✅ **Graceful Shutdown** - CAN commands to save state before power-off

**BSEC Configuration:**
- Config blob: `bme680_iaq_33v_3s_4d` (3.3V, LP mode/3s, 4-day calibration)
- Sample rate: BSEC_SAMPLE_RATE_LP (3-second intervals)
- 8 virtual sensors subscribed (see bme680_bsec.c:370-382)

**Key Technical Details:**

**I2C Addresses:**
- Primary: 0x77 (default for most breakout boards)
- Alternate: 0x76 (configurable via SDO pin)
- Auto-detection tries both addresses

**Reading Flow** [bme680_bsec.c:492-719]:
1. Call `bsec_sensor_control()` to get BSEC-requested settings
2. Apply sensor configuration (T/H/P oversampling, heater profile)
3. Trigger forced mode measurement
4. Wait for measurement completion (~1140ms with gas heater)
5. Read raw sensor data (T/H/P/Gas)
6. Feed data to BSEC library via `bsec_do_steps()`
7. Extract BSEC virtual sensor outputs (IAQ, CO2, VOC, etc.)
8. Auto-save state to NVS every 4 hours

**Data Accuracy:**
- Temperature: ±1.0°C (typ), ±1.5°C (max) - heat-compensated
- Humidity: ±3% RH (typ)
- Pressure: ±1.0 hPa (typ)
- IAQ: 0-500 (0-50=good, 51-100=average, 101-150=little bad, 151-200=bad, 201-300=worse, 301-500=very bad)
- CO2 equivalent: 400-60000 ppm (estimated)
- Breath VOC: 0.5-1000+ ppm (estimated)

**NVS State Management:**
- Namespace: `bme68x_state`
- Auto-saves BSEC calibration state every 4 hours
- Persists across reboots for faster IAQ convergence
- State includes gas sensor baseline and calibration history
- CAN command 0x112/0x113 triggers immediate save

**IAQ Calibration Timeline:**
- **Accuracy 0:** Uncalibrated (first 5 minutes)
- **Accuracy 1:** Low accuracy (5-30 minutes) - usable but not optimal
- **Accuracy 2:** Medium accuracy (30+ minutes) - good for most applications
- **Accuracy 3:** High accuracy (hours to days) - fully calibrated baseline
- Calibration persists in NVS, so subsequent boots start with saved accuracy

**Critical Bug Fixes (Oct 2025):**

1. ✅ **BSEC Subscription Fixed** - Struct field order was incorrect
   - **Wrong:** `{BSEC_OUTPUT_IAQ, BSEC_SAMPLE_RATE_LP}`
   - **Correct:** `{ .sample_rate = BSEC_SAMPLE_RATE_LP, .sensor_id = BSEC_OUTPUT_IAQ }`
   - Used designated initializers to prevent field order mismatch
   - Location: bme680_bsec.c:370-382

2. ✅ **NVS Flash Initialization** - NVS was never initialized
   - Added `nvs_flash_init()` in app_main() before sensor initialization
   - Without this, all calibration save/load operations silently failed
   - Location: main.c:317-325

3. ✅ **Hardware Defect Detected** - Original BME680 board had defective gas sensor
   - Symptoms: Gas measurements timed out, CTRL_MEAS corrupted to 0x00
   - Diagnostics: Extensive register dumps and timing analysis
   - Solution: Replaced BME680 board - new sensor works perfectly
   - Measurement time with working sensor: 1142ms (expected for 1000ms heater duration)

### A/B OTA Firmware Updates

The firmware supports over-the-air updates via CAN bus using an A/B partition scheme:

**Flash Layout (4MB):**
```
NVS:     0x9000  (24KB)  - Calibration, node ID storage
OTA Data: 0x10000 (8KB)  - Tracks active partition
OTA_0:   0x20000 (~1.94MB) - Firmware slot A
OTA_1:   0x210000 (~1.94MB) - Firmware slot B
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

**OTA Protocol (CAN):**
- OTA base: 0x700, spacing 0x10 per node
- 6-byte chunks per CAN frame with sequence numbers
- CRC32 verification of complete firmware image
- Transfer speed: ~3 KB/s
- Session timeout: 5 minutes, max 3 retries per chunk
- Partition switch verification after reboot (pre/post info query)
- Batch update waits for each node to reboot before proceeding to next
- Use `can-sensor-tool update firmware.bin` for managed updates

### CAN Protocol

**Node Addressing:**
- Base address: 0x100, spacing: 0x20 (32 IDs per node), max 16 nodes (0-15)
- Sensor data offsets: 0x00-0x0F, Control command offsets: 0x10-0x1F
- OTA: 0x700 base, 0x10 spacing (unchanged)

**Control Messages (RX - from master to ESP32, Node 0 example):**
- `0x110`: STOP - Stop sensor transmission
- `0x111`: START - Start sensor transmission (auto-start at boot if no CAN activity)
- `0x112`: SHUTDOWN - Graceful shutdown (save BSEC state, stop TX, keep ESP32 running)
- `0x113`: REBOOT - Save BSEC state and reboot ESP32
- `0x114`: FACTORY_RESET - Clear BSEC calibration (factory default) and reboot
- `0x115`: SET_NODE_ID - Change node ID (byte 0 = new ID, 0-15)
- `0x116`: GET_DEVICE_INFO - Request device info
- `0x118`: DISCOVERY_PING - Ping for device discovery
- `0x11A`: IDENTIFY - Blink onboard LED for 5 seconds (GPIO8, active-low)

**Response Messages (TX - from ESP32 to master, Node 0 example):**
- `0x117`: DEVICE_INFO_RESPONSE - Firmware version, sensor flags, partition info
- `0x119`: DISCOVERY_PONG - Response to discovery ping

**Data Messages (TX - from ESP32 to master, Node 0 example):**
- `0x100`: Ambient light data (1 Hz, VEML7700, OPT3001, or OPT4001)
- `0x101`: BME680 environmental data (0.33 Hz, T/H/P)
- `0x102`: BME680 air quality data (0.33 Hz, IAQ/CO2/VOC)
- `0x103-0x106`: BME688 gas selectivity classes 1-4 (future)
- `0x107-0x108`: mm-wave presence detection (future)
- `0x10F`: System status

**Message Format offset 0x00 (Ambient Light):**
```
Byte 0-2: Lux value (uint24_t, little-endian, 0-16,777,215)
Byte 3:   Status (0x00=OK, 0x01=Error)
Byte 4:   Sequence counter (rolls over)
Byte 5:   Config index (0-20=VEML7700, 100-111=OPT4001, 200-211=OPT3001)
Byte 6-7: Checksum (sum of bytes 0-5, uint16_t LE)
```

**Message Format offset 0x01 (Environmental - T/H/P):**
```
Byte 0-1: Temperature (int16_t, 0.1°C resolution, -40.0 to 85.0°C)
Byte 2-3: Humidity (uint16_t, 0.1% resolution, 0.0 to 100.0%)
Byte 4-5: Pressure (uint16_t, 0.1 hPa resolution, 300.0 to 1100.0 hPa)
Byte 6:   Reserved (0x00)
Byte 7:   Status (0x00=OK, 0x01=Error)
```

**Message Format offset 0x02 (Air Quality - IAQ/CO2/VOC):**
```
Byte 0-1: IAQ (uint16_t, 0-500 index)
Byte 2:   IAQ Accuracy (uint8_t, 0-3: 0=uncalibrated, 3=fully calibrated)
Byte 3-4: CO2 equivalent (uint16_t, ppm)
Byte 5-6: Breath VOC equivalent (uint16_t, ppm)
Byte 7:   Status (0x00=OK, 0x01=Error)
```

**Message Format offset 0x17 (Device Info Response):**
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

**Control Command Usage Examples:**
```bash
# Normal operation control (Node 0)
cansend can0 111#              # Start transmission
cansend can0 110#              # Stop transmission

# Graceful shutdown before power-off
cansend can0 112#              # Save calibration, stop TX
# (wait for "BSEC state saved" message, then safe to power off)

# Reboot ESP32 (preserves calibration)
cansend can0 113#              # Save state and reboot

# Factory reset (clear calibration)
cansend can0 114#              # Reset to factory defaults, reboot

# Identify node (blink LED)
cansend can0 11A#              # Blink onboard LED for 5 seconds
```

### Pin Assignments

```
GPIO4:  CAN TX
GPIO5:  CAN RX
GPIO6:  I2C SDA (Shared bus for all I2C sensors)
GPIO7:  I2C SCL (Shared bus for all I2C sensors)
GPIO8:  Onboard LED (active-low, used for identify blink)
```

**I2C Bus Sharing:**
- VEML7700 @ 0x10 OR OPT4001/OPT3001 @ 0x44 (Ambient light - only one at a time)
- BME680/BME688 @ 0x76 or 0x77 (Environmental sensor)
- All sensors share the same I2C bus (I2C_NUM_0)
- Pull-up resistors: 4.7kΩ recommended

### Resource Usage

- **Flash:** ~295 KB (with OPT4001 + VEML7700 + BME680 + BSEC)
- **SRAM:** ~90 KB (out of 400 KB available)
- **CPU Load:** <10% @ 160 MHz (both sensors active)
- **Task Stacks:**
  - sensor_poll_task: 3 KB (ALS sensor)
  - bme680_sensor_task: 10 KB (BSEC library requires large stack)
  - twai_transmit_task: 5 KB
  - twai_receive_task: 3 KB
  - twai_control_task: 2 KB (self-deletes after start)

## Development Guidelines

### When Modifying VEML7700 Driver

1. **Auto-ranging changes:** Test across full dynamic range (0-120,000 lux)
   - Verify no oscillation between configs
   - Check settling time is adequate
   - Monitor with: `candump can0 | grep 100`

2. **Configuration table:** Maintain order (most sensitive → least sensitive)
   - Index 0: Highest gain/longest integration (2x/800ms)
   - Index 17: Lowest gain/shortest integration (1/8x/200ms)

3. **Filter modifications:** Maintain buffer size power-of-2 for efficiency
   - Current: 8 samples
   - Trade-off: larger = smoother but slower response

### When Modifying OPT4001 Driver

1. **I2C byte order:** OPT4001 uses **BIG ENDIAN** (MSB first)
   - Write: MSB byte, then LSB byte
   - Read: MSB byte, then LSB byte
   - Different from many sensors!

2. **Lux calculation constant:** Depends on package variant
   - SOT-5x3: `0.0004375` (current implementation)
   - PICOSTAR: `0.0003125`
   - Formula: `lux = mantissa × 2^exponent × constant`

3. **Register structure:** Result is split across two 16-bit registers
   - Register 0x00 (MSB): bits[15:12]=exponent, bits[11:0]=mantissa[19:8]
   - Register 0x01 (LSB): bits[15:8]=mantissa[7:0], bits[7:0]=counter+CRC
   - Must read both registers to get full 20-bit mantissa

4. **Testing:** Verify against reference lux meter
   - Target accuracy: ±2% across range
   - Test at: dim indoor (~100 lux), bright indoor (~1000 lux), outdoor (~10K+ lux)

### When Modifying BME680 Driver

**Current Full IAQ Mode (Production):**

1. **Sensor configuration is BSEC-DRIVEN** - not fixed
   - Call `bsec_sensor_control()` before EVERY measurement
   - Apply BSEC-provided settings (T/H/P oversampling, heater profile)
   - BSEC timing is CRITICAL - respect `next_call` timing (error 100 if late)
   - Gas heater: 380°C for 1000ms (BSEC-controlled)

2. **Values come from BSEC virtual sensors** - not raw sensor
   - Feed raw data to BSEC via `bsec_do_steps()`
   - Extract outputs from `bsec_output_t` array
   - Handle all 8 subscribed virtual sensors (IAQ, CO2, VOC, T, H, P, status flags)
   - Use heat-compensated temperature (not raw temperature)

3. **Struct initialization MUST use designated initializers**
   - **CRITICAL:** Field order in `bsec_sensor_configuration_t` is {sample_rate, sensor_id}
   - **Wrong:** `{BSEC_OUTPUT_IAQ, BSEC_SAMPLE_RATE_LP}` (will fail with warning 10)
   - **Correct:** `{ .sample_rate = BSEC_SAMPLE_RATE_LP, .sensor_id = BSEC_OUTPUT_IAQ }`
   - Location: bme680_bsec.c:370-382

4. **NVS initialization is MANDATORY**
   - Must call `nvs_flash_init()` in app_main() BEFORE sensor init
   - Without NVS, all state save/load operations silently fail
   - Calibration will be lost on every reboot
   - Location: main.c:317-325

5. **Calibration takes time**
   - IAQ accuracy 0 = uncalibrated (first 5 minutes) - values unreliable
   - IAQ accuracy 1 = low accuracy (5-30 minutes) - usable but not optimal
   - IAQ accuracy 2 = medium accuracy (30+ minutes) - good for most applications
   - IAQ accuracy 3 = high accuracy (hours to days) - fully calibrated baseline
   - State persists in NVS, so don't clear NVS during testing
   - Use CAN command 0x112 or 0x113 to save state before power-off

6. **Stack size is critical**
   - BME680 task requires 10KB stack minimum
   - BSEC library uses significant stack space
   - Stack overflow will cause silent crashes

7. **Hardware defects can occur**
   - Symptoms: Gas measurements timeout, CTRL_MEAS corrupted to 0x00
   - Diagnostics: Add register dumps (CTRL_GAS_0/1, MEAS_STATUS_0)
   - If gas sensor circuit is defective, replace BME680 board
   - Expected measurement time: ~1140ms (with 1000ms heater duration)

### When Modifying CAN Protocol

1. **Preserve message format:** Many receivers may depend on current structure
2. **Update checksum calculation** if adding/removing data fields
3. **Test with:** `candump can0 -d` (decode data)

### When Adding Features

The codebase uses a **modular, queue-based architecture** designed for expansion:

**Adding a New Sensor:**
1. Create `sensor_name_driver.c/h` following the pattern in `veml7700_driver.c/h`
2. Add sensor type to `sensor_id_t` enum in `sensor_common.h`
3. Add data fields to the union in `sensor_data_t` structure
4. Create a sensor task or integrate into `sensor_poll_task`
5. Add CAN message formatting function in `can_protocol.c/h`
6. Update `twai_transmit_task` to handle new sensor data
7. Update `CMakeLists.txt` to include new source files

**Existing Modular Components:**
- `veml7700_driver.c/h`: VEML7700 sensor (I2C, software auto-ranging)
- `opt4001_driver.c/h`: OPT4001 sensor (I2C, hardware auto-ranging)
- `opt3001_driver.c/h`: OPT3001 sensor (I2C, hardware auto-ranging)
- `bme680_bsec.c/h`: BME680/BME688 with BSEC 2.x integration
- `als_driver.c/h`: Unified ALS abstraction with auto-detection
- `ota_handler.c/h`: A/B OTA firmware updates over CAN
- `sensor_common.h`: Shared data structures and enums
- `can_protocol.c/h`: CAN message formatting and protocol definitions
- `firmware_config.h`: Feature flags and version constants
- `main.c`: Task coordination and queue management

See `EXPANSION_GUIDE.md` for detailed multi-sensor expansion plan (BME680, LD2410, BME688, MQ-3)

### Common Issues

**I2C timeouts:**
- Check wiring (SDA/SCL not swapped)
- Verify pull-up resistors (4.7kΩ recommended)
- VEML7700 address: 0x10 (cannot be changed)

**CAN messages not seen on bus:**
- Verify CAN transceiver (SN65HVD230) has proper supply voltage
- Check termination resistors (120Ω at both ends)
- Bitrate must match: 500 kbps

**Auto-ranging unstable:**
- Check for light source flickering (AC powered lights)
- Increase `SETTLE_READINGS_REQUIRED` [veml7700_driver.c:46]
- Increase `MOVING_AVG_SAMPLES` [veml7700_driver.c:49]

**Serial monitor crashes (ESP32-C6 + ESP-IDF 5.5):**
- Known issue with USB-Serial/JTAG
- Use CAN monitoring instead: functionality is unaffected

**Sensor maxes out at ~10K lux in sunlight:**
- Verify firmware has all 21 configs (check compile timestamp)
- Check monitor logs for saturation ramp-up messages
- Should see: `Config switch: X→Y, reason=SATURATION_RAMP`
- Final config should be 18-20 for bright sunlight (>50K lux)

**High-lux readings don't match reference meter:**
- Check calibration factor for >10K lux range (should be 0.915x)
- Ensure sensor and reference meter at same position/angle
- Note: ±5% variance is normal due to diffuser properties and spectral differences

**OPT4001 readings always zero:**
- Check I2C byte order (must be BIG ENDIAN - MSB first)
- Verify config register write succeeded (read back to confirm)
- Check device ID read correctly (should be 0x121 after masking bits 11:0)
- Ensure 900ms delay after config write (for first conversion)

**OPT4001 readings don't match reference meter:**
- Verify package constant: SOT-5x3 = 0.0004375, PICOSTAR = 0.0003125
- Check formula: `lux = mantissa × 2^exponent × constant`
- Compare raw register values (MSB/LSB) with reference implementation
- Target accuracy: ±2% across range

**Stack overflow in sensor_poll_task:**
- OPT4001 uses `powf()` which requires more stack
- Increase stack size to 3072 bytes minimum
- Check in `main.c`: `xTaskCreate(sensor_poll_task, "SENSOR_POLL", 3072, ...)`

**BME680 BSEC subscription fails (warning 10):**
- Root cause: Struct field order mismatch in initialization
- Check `bsec_sensor_configuration_t` uses designated initializers
- Correct format: `{ .sample_rate = X, .sensor_id = Y }`
- See fix in bme680_bsec.c:370-382

**BME680 calibration not persisting across reboots:**
- Check if NVS flash is initialized in app_main()
- Must call `nvs_flash_init()` before sensor init
- Verify "BSEC state saved to NVS" message appears every 4 hours
- Use CAN command 0x113 to test save/reboot cycle

**BME680 gas measurements timeout:**
- Check measurement completion time (should be ~1140ms)
- If >3 seconds with no completion, likely hardware defect
- Add diagnostic register dumps (CTRL_MEAS, MEAS_STATUS_0)
- If CTRL_MEAS corrupts to 0x00, replace BME680 board

**IAQ/CO2/VOC values stuck at defaults (50/500/0.5):**
- Check IAQ accuracy - must be >0 for valid data
- Accuracy 0 = uncalibrated (wait 5+ minutes)
- Verify gas heater is enabled (should see 380°C in logs)
- Check BSEC virtual sensor subscription succeeded (no warning 10)

**OTA update fails or stalls:**
- Ensure node is responding: `can-sensor-tool ping`
- Try with longer chunk delay: `can-sensor-tool --chunk-delay=10 update firmware.bin`
- Check for CAN bus errors: `ip -s link show can0`
- Verify firmware binary size fits in partition (~1.94 MB max)
- Check for OTA session timeout (5 min idle = abort)

**OTA partition unchanged after update:**
- Update summary shows "partition unchanged (ota_X)" = firmware didn't activate
- Check if firmware calls `esp_ota_mark_app_valid()` on boot
- Bootloader may have rolled back to previous slot (3 failed boot attempts)
- Verify firmware CRC matches: tool checks CRC32 before activating

**Node not discovered by can-sensor-tool:**
- Verify CAN interface is up: `ip link show can0`
- Check node ID hasn't been changed: try `can-sensor-tool --node=N ping` for N=0-15
- Ensure firmware is running (not stuck in bootloader)

**"Unknown BSEC output sensor_id=12/13" warnings:**
- These are stabilization_status and run_in_status outputs
- Not critical for operation, just status flags
- Can be handled at DEBUG level or ignored
- Fixed in latest version (bme680_bsec.c:795-803)

## Testing Checklist

### Unit Testing
- [ ] I2C communication (read/write registers)
- [ ] Sensor auto-detection (VEML7700 @ 0x10, OPT4001/OPT3001 @ 0x44, BME680 @ 0x76/0x77)
- [ ] All 21 VEML7700 configurations apply correctly
- [ ] OPT4001 device ID verification (0x121)
- [ ] OPT4001 config register write/readback
- [ ] BME680 device ID verification (variant 0x00 or 0x01)
- [ ] BSEC library initialization and config loading
- [ ] BSEC virtual sensor subscription (8 outputs)
- [ ] Checksum calculation for CAN messages
- [ ] Moving average filter output (VEML7700)
- [ ] 3-byte lux encoding/decoding
- [ ] NVS flash initialization

### Integration Testing - VEML7700
- [ ] Auto-ranging across 0-120,000 lux
- [ ] Saturation ramp-up in bright light (watch for rapid 8→9→...→20 progression)
- [ ] No oscillation between configs
- [ ] Smooth transitions: indoor (500 lux) → outdoor (70K lux)
- [ ] Calibration accuracy vs reference meter (indoor and sunlight)
- [ ] Config index 0-20 reported correctly in CAN messages (offset 0x00)

### Integration Testing - OPT4001
- [ ] Hardware auto-ranging across 0-2.2M lux
- [ ] Accuracy vs reference meter: ±2% at 100 lux, 1K lux, 10K+ lux
- [ ] Range transitions smooth (config 100-111)
- [ ] Config index 100-111 reported correctly in CAN messages (offset 0x00)
- [ ] No stack overflow with powf() usage

### Integration Testing - BME680
- [ ] Sensor auto-detection at both addresses (0x76/0x77)
- [ ] BSEC subscription succeeds (no warning 10)
- [ ] Gas measurements complete in ~1140ms (not timeout)
- [ ] T/H/P values reasonable (T: room temp ±1.5°C, H: 20-80%, P: 980-1030 hPa)
- [ ] IAQ calibration progresses (accuracy 0→1→2→3 over time)
- [ ] CO2 and VOC values update (not stuck at 500/0.5)
- [ ] Environmental data CAN messages (offset 0x01) at 0.33 Hz
- [ ] Air quality CAN messages (offset 0x02) at 0.33 Hz
- [ ] NVS state save every 4 hours
- [ ] Calibration persists across reboot (test with reboot command, offset 0x13)

### Integration Testing - CAN Commands
- [ ] START command (offset 0x11) - transmission begins
- [ ] STOP command (offset 0x10) - transmission stops
- [ ] SHUTDOWN command (offset 0x12) - state saved, TX stops, ESP32 keeps running
- [ ] REBOOT command (offset 0x13) - state saved, ESP32 reboots, calibration restored
- [ ] FACTORY_RESET command (offset 0x14) - calibration cleared, IAQ accuracy returns to 0
- [ ] SET_NODE_ID command (offset 0x15) - node ID changes and persists in NVS
- [ ] GET_DEVICE_INFO command (offset 0x16) - returns firmware version and sensor flags
- [ ] DISCOVERY_PING command (offset 0x18) - node responds with pong
- [ ] IDENTIFY command (offset 0x1A) - onboard LED blinks for 5 seconds
- [ ] IDENTIFY re-trigger - sending again restarts the 5-second timer

### Integration Testing - OTA
- [ ] OTA update via can-sensor-tool: `can-sensor-tool update firmware.bin`
- [ ] Partition alternates between OTA_0 and OTA_1
- [ ] Partition switch verified automatically (pre/post info query)
- [ ] Batch OTA: `can-sensor-tool --all update firmware.bin` (sequential, wait for reboot)
- [ ] Batch OTA summary shows partition transitions per node
- [ ] Automatic rollback on corrupted firmware
- [ ] OTA session timeout after 5 minutes of inactivity
- [ ] CRC32 verification rejects corrupted images

### Integration Testing - Multi-Sensor
- [ ] All sensors coexist on shared I2C bus
- [ ] No I2C contention or timeouts
- [ ] CAN messages for all sensors transmitted correctly
- [ ] Sensor type correctly identified from config index

### Reliability Testing
- [ ] 24-hour continuous operation with all sensors
- [ ] No memory leaks (check heap)
- [ ] Handles sensor disconnect gracefully
- [ ] CAN bus error recovery
- [ ] IAQ calibration reaches accuracy 3 within expected timeframe
- [ ] State persistence across multiple reboots

## Configuration Options

### In sdkconfig (via `idf.py menuconfig`)

**Critical settings:**
- Component config → ESP32C3/C6-Specific → CPU frequency: **160 MHz**
- Component config → FreeRTOS → Tick rate: **100 Hz**
- Component config → Log output → Default log verbosity: **Info** (Debug for development)

**For production:**
- Disable verbose logging (saves flash)
- Enable stack overflow checking: Component config → FreeRTOS → Check for stack overflow
- Consider watchdog timeout adjustments

### In Code Constants

**main.c:**
```c
#define CAN_DATA_PERIOD_MS      1000    // CAN transmission rate [main.c:35]
#define SENSOR_QUEUE_DEPTH      20      // Sensor data queue size [main.c:36]
```

**veml7700_driver.c:**
```c
#define MOVING_AVG_SAMPLES      8       // Filter size
#define SETTLE_READINGS_REQUIRED 3      // Stability checks before config change
#define SATURATED_HIGH          60000   // Auto-ranging thresholds
#define OPTIMAL_HIGH            40000
#define OPTIMAL_LOW             10000
#define TOO_LOW                 2000
```

## Future Expansion

The codebase has been refactored with a **modular, queue-based architecture** ready for multi-sensor expansion:

**Phase 0 (Completed - Modular Refactoring):**
- ✅ Extracted VEML7700 into separate driver module
- ✅ Implemented queue-based data flow
- ✅ Created shared sensor data structures
- ✅ Separated CAN protocol formatting
- ✅ Verified functionality (all tests passing)

**Phase 0.5 (Completed - High-Lux Sunlight Support):**
- ✅ Extended sensor range from 15K to 120K lux
- ✅ Added 3 new VEML7700 configurations (configs 18-20)
- ✅ Implemented saturation ramp-up auto-ranging algorithm
- ✅ Upgraded CAN protocol from 2-byte (65K max) to 3-byte (16M max) lux encoding
- ✅ Added high-intensity calibration (>10K lux range)
- ✅ Validated against reference lux meter (±1.4% accuracy at 70K lux)
- ✅ Updated calibration utility for 3-byte lux parsing

**Phase 1 (Completed - BME680/BME688 Integration):**
- ✅ Added BME680/BME688 environmental sensor with BSEC 2.6.1.0
- ✅ Full IAQ mode with 8 virtual sensor outputs
- ✅ NVS state persistence for calibration
- ✅ CAN messages for T/H/P (offset 0x01) and IAQ/CO2/VOC (offset 0x02)
- ✅ Graceful shutdown commands (offsets 0x12, 0x13, 0x14)

**Additional Completed Features:**
- ✅ OPT3001 ambient light sensor support (0-83K lux, config index 200-211)
- ✅ A/B OTA firmware updates over CAN bus with automatic rollback
- ✅ `can-sensor-tool` CLI for node discovery, monitoring, OTA, and management
- ✅ Multi-node addressing with configurable node IDs (0-15)
- ✅ Device discovery and info commands (0x115-0x119)

**Next Phases (Planned):**
- Phase 2: Add HLK-LD2410 human presence radar (UART)
- Phase 3: Add BME688 air quality sensor (enhanced BSEC)
- Phase 4: Add MQ-3 alcohol/VOC gas sensor (ADC)

See `EXPANSION_GUIDE.md` for detailed implementation plans including:
- Hardware pin assignments
- Resource budgets (SRAM, Flash, CPU)
- Extended CAN protocol (offsets 0x00-0x0F sensor data, 0x10-0x1F control)
- Task coordination strategies
- Testing procedures
