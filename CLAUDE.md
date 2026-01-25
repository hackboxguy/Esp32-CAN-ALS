# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Summary

ESP32-C3/C6 automotive CAN node with **multi-sensor support**: ambient light sensors (VEML7700/OPT4001) and environmental sensors (BME680/BME688). Production-ready implementation with runtime auto-detection, intelligent auto-ranging, and modular queue-based architecture designed for expansion.

**Sensor Support:**
- **Ambient Light:** VEML7700 (0-120K lux, calibrated) OR OPT4001 (0-2.2M lux, factory calibrated)
- **Environmental:** BME680/BME688 with BSEC 2.x (temperature, humidity, pressure, air quality)

## Build and Development Commands

### ESP32 Firmware

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

# Control commands
cansend can0 0A1#        # Start transmission
cansend can0 0A0#        # Stop transmission
cansend can0 0A8#        # Graceful shutdown (save state)
cansend can0 0A9#        # Reboot ESP32 (save state first)
cansend can0 0AA#        # Factory reset (clear calibration)
```

### Testing

```bash
# Run firmware and monitor serial output
idf.py monitor

# In separate terminal, monitor CAN bus
candump can0 -t z

# Test basic functionality
cansend can0 0A1#        # Start transmission
# Should see:
#   0x0A2 @ 1 Hz (ambient light - VEML7700 or OPT4001)
#   0x0A3 @ 0.33 Hz (BME680 environmental - T/H/P)
#   0x0A4 @ 0.33 Hz (BME680 air quality - IAQ/CO2/VOC)

# Test graceful shutdown
cansend can0 0A8#        # Shutdown (saves BSEC state)
# Wait for "BSEC state saved to NVS" message

# Test reboot with state persistence
cansend can0 0A9#        # Reboot (saves and reboots)
# Watch for IAQ accuracy to persist after reboot

# Test factory reset
cansend can0 0AA#        # Factory reset
# IAQ accuracy should return to 0 after reboot
```

## Code Architecture

### High-Level Structure (Modular)

```
main/
├── main.c (~420 lines) - Task coordination and initialization
│   ├── sensor_poll_task: Periodic sensor reading (1 Hz)
│   ├── twai_receive_task: CAN command reception (START/STOP)
│   ├── twai_transmit_task: CAN message transmission
│   └── twai_control_task: Auto-start trigger (one-shot)
│
├── sensor_common.h (60 lines) - Shared data structures
│   ├── sensor_id_t: Sensor enumeration
│   ├── sensor_data_t: Unified sensor data union
│   └── Status codes and constants
│
├── als_driver.c/h (~270 lines) - Ambient Light Sensor abstraction layer
│   ├── Runtime auto-detection (I2C address probing)
│   ├── Unified API for VEML7700 or OPT4001
│   ├── I2C bus initialization
│   └── Sensor type identification via config index
│
├── veml7700_driver.c/h (~550 lines) - VEML7700 sensor module
│   ├── I2C communication functions
│   ├── Configuration table (21 configs: gain × integration time)
│   ├── Intelligent auto-ranging algorithm with saturation ramp-up
│   ├── Moving average filter (8 samples)
│   └── Calibration correction (piecewise linear, 0-120K lux range)
│
├── opt4001_driver.c/h (~230 lines) - OPT4001 sensor module
│   ├── I2C communication (big-endian)
│   ├── Hardware auto-ranging (12 ranges, 0-11)
│   ├── 800ms integration time (highest accuracy)
│   ├── Factory-calibrated (SOT-5x3 package)
│   └── No filtering needed (stable readings)
│
├── bme680_bsec.c/h (~1050 lines) - BME680/BME688 environmental sensor with BSEC
│   ├── Runtime auto-detection (I2C address 0x76/0x77)
│   ├── BSEC 2.6.1.0 library integration (IAQ/CO2/VOC enabled)
│   ├── Virtual sensor subscription with 8 BSEC outputs
│   ├── Fixed 3-second reading interval (LP mode)
│   ├── Gas heater enabled (380°C, 1000ms for IAQ measurement)
│   ├── NVS state management (auto-saves every 4 hours)
│   └── Calibration persistence across reboots
│
└── can_protocol.c/h (~300 lines) - CAN message formatting
    ├── Message ID definitions
    ├── can_format_veml7700_message()
    └── Checksum calculation
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

The firmware supports two ambient light sensors with **runtime auto-detection**:

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

**Auto-Detection Logic** [als_driver.c]:
1. Initialize I2C bus
2. Probe address 0x10 (VEML7700)
   - If found → initialize VEML7700 driver
3. Probe address 0x44 (OPT4001)
   - If found → initialize OPT4001 driver
4. Use detected sensor transparently via unified API

**Sensor Identification:**
- CAN clients can identify sensor type via config index:
  - Config 0-20 = VEML7700
  - Config 100-111 = OPT4001

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
- CAN command 0x0A8/0x0A9 triggers immediate save

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

### CAN Protocol

**Control Messages (RX - from master to ESP32):**
- `0x0A0`: STOP - Stop sensor transmission
- `0x0A1`: START - Start sensor transmission (auto-start at boot if no CAN activity)
- `0x0A8`: SHUTDOWN - Graceful shutdown (save BSEC state, stop TX, keep ESP32 running)
- `0x0A9`: REBOOT - Save BSEC state and reboot ESP32
- `0x0AA`: FACTORY_RESET - Clear BSEC calibration (factory default) and reboot

**Data Messages (TX - from ESP32 to master):**
- `0x0A2`: Ambient light data (1 Hz, VEML7700 or OPT4001)
- `0x0A3`: BME680 environmental data (0.33 Hz, T/H/P)
- `0x0A4`: BME680 air quality data (0.33 Hz, IAQ/CO2/VOC)

**Message Format 0x0A2 (Ambient Light):**
```
Byte 0-2: Lux value (uint24_t, little-endian, 0-16,777,215)
Byte 3:   Status (0x00=OK, 0x01=Error)
Byte 4:   Sequence counter (rolls over)
Byte 5:   Config index (0-20=VEML7700, 100-111=OPT4001)
Byte 6-7: Checksum (sum of bytes 0-5, uint16_t LE)
```

**Message Format 0x0A3 (Environmental - T/H/P):**
```
Byte 0-1: Temperature (int16_t, 0.1°C resolution, -40.0 to 85.0°C)
Byte 2-3: Humidity (uint16_t, 0.1% resolution, 0.0 to 100.0%)
Byte 4-5: Pressure (uint16_t, 0.1 hPa resolution, 300.0 to 1100.0 hPa)
Byte 6:   Reserved (0x00)
Byte 7:   Status (0x00=OK, 0x01=Error)
```

**Message Format 0x0A4 (Air Quality - IAQ/CO2/VOC):**
```
Byte 0-1: IAQ (uint16_t, 0-500 index)
Byte 2:   IAQ Accuracy (uint8_t, 0-3: 0=uncalibrated, 3=fully calibrated)
Byte 3-4: CO2 equivalent (uint16_t, ppm)
Byte 5-6: Breath VOC equivalent (uint16_t, ppm)
Byte 7:   Status (0x00=OK, 0x01=Error)
```

**Control Command Usage Examples:**
```bash
# Normal operation control
cansend can0 0A1#              # Start transmission
cansend can0 0A0#              # Stop transmission

# Graceful shutdown before power-off
cansend can0 0A8#              # Save calibration, stop TX
# (wait for "BSEC state saved" message, then safe to power off)

# Reboot ESP32 (preserves calibration)
cansend can0 0A9#              # Save state and reboot

# Factory reset (clear calibration)
cansend can0 0AA#              # Reset to factory defaults, reboot
```

### Pin Assignments

```
GPIO4:  CAN TX
GPIO5:  CAN RX
GPIO6:  I2C SDA (Shared bus for all I2C sensors)
GPIO7:  I2C SCL (Shared bus for all I2C sensors)
```

**I2C Bus Sharing:**
- VEML7700 @ 0x10 OR OPT4001 @ 0x44 (Ambient light - only one should be connected)
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
   - Monitor with: `candump can0 | grep 0A2`

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
   - Use CAN command 0x0A8 or 0x0A9 to save state before power-off

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
- `veml7700_driver.c/h`: VEML7700 sensor (I2C)
- `sensor_common.h`: Shared data structures and enums
- `can_protocol.c/h`: CAN message formatting utilities
- `main.c`: Task coordination and queue management

See `EXPANSION_GUIDE.md` for detailed multi-sensor expansion plan (BME680, LD2410, BME688, MQ-3)

### Common Issues

**I2C timeouts:**
- Check wiring (SDA/SCL not swapped)
- Verify pull-up resistors (4.7kΩ recommended)
- VEML7700 address: 0x10 (cannot be changed)

**CAN messages not seen on bus:**
- Verify TJA1050 has 5V supply (required for proper signaling)
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
- Use CAN command 0x0A9 to test save/reboot cycle

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

**"Unknown BSEC output sensor_id=12/13" warnings:**
- These are stabilization_status and run_in_status outputs
- Not critical for operation, just status flags
- Can be handled at DEBUG level or ignored
- Fixed in latest version (bme680_bsec.c:795-803)

## Testing Checklist

### Unit Testing
- [ ] I2C communication (read/write registers)
- [ ] Sensor auto-detection (VEML7700 @ 0x10, OPT4001 @ 0x44, BME680 @ 0x76/0x77)
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
- [ ] Config index 0-20 reported correctly in CAN messages (0x0A2)

### Integration Testing - OPT4001
- [ ] Hardware auto-ranging across 0-2.2M lux
- [ ] Accuracy vs reference meter: ±2% at 100 lux, 1K lux, 10K+ lux
- [ ] Range transitions smooth (config 100-111)
- [ ] Config index 100-111 reported correctly in CAN messages (0x0A2)
- [ ] No stack overflow with powf() usage

### Integration Testing - BME680
- [ ] Sensor auto-detection at both addresses (0x76/0x77)
- [ ] BSEC subscription succeeds (no warning 10)
- [ ] Gas measurements complete in ~1140ms (not timeout)
- [ ] T/H/P values reasonable (T: room temp ±1.5°C, H: 20-80%, P: 980-1030 hPa)
- [ ] IAQ calibration progresses (accuracy 0→1→2→3 over time)
- [ ] CO2 and VOC values update (not stuck at 500/0.5)
- [ ] Environmental data CAN messages (0x0A3) at 0.33 Hz
- [ ] Air quality CAN messages (0x0A4) at 0.33 Hz
- [ ] NVS state save every 4 hours
- [ ] Calibration persists across reboot (test with 0x0A9 command)

### Integration Testing - CAN Commands
- [ ] START command (0x0A1) - transmission begins
- [ ] STOP command (0x0A0) - transmission stops
- [ ] SHUTDOWN command (0x0A8) - state saved, TX stops, ESP32 keeps running
- [ ] REBOOT command (0x0A9) - state saved, ESP32 reboots, calibration restored
- [ ] FACTORY_RESET command (0x0AA) - calibration cleared, IAQ accuracy returns to 0

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
- ✅ CAN messages 0x0A3 (T/H/P) and 0x0A4 (IAQ/CO2/VOC)
- ✅ Graceful shutdown commands (0x0A8, 0x0A9, 0x0AA)

**Next Phases (Planned):**
- Phase 2: Add HLK-LD2410 human presence radar (UART)
- Phase 3: Add BME688 air quality sensor (enhanced BSEC)
- Phase 4: Add MQ-3 alcohol/VOC gas sensor (ADC)

See `EXPANSION_GUIDE.md` for detailed implementation plans including:
- Hardware pin assignments
- Resource budgets (SRAM, Flash, CPU)
- Extended CAN protocol (0x0A3-0x0A7)
- Task coordination strategies
- Testing procedures
