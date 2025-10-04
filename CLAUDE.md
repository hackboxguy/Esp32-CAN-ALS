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

# Send commands
cansend can0 0A0#        # Stop transmission
cansend can0 0A1#        # Start transmission
```

### Testing

```bash
# Run firmware and monitor serial output
idf.py monitor

# In separate terminal, monitor CAN bus
candump can0 -t z

# Test basic functionality
cansend can0 0A1#        # Should see 0x0A2 messages at 1 Hz
```

## Code Architecture

### High-Level Structure (Modular)

```
main/
├── main.c (~250 lines) - Task coordination and initialization
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
├── als_driver.c/h (~150 lines) - Ambient Light Sensor abstraction layer
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
├── bme680_bsec.c/h (~800 lines) - BME680/BME688 environmental sensor with BSEC
│   ├── Runtime auto-detection (I2C address 0x76/0x77)
│   ├── BSEC 2.6.1.0 library integration
│   ├── **Simplified mode:** Direct T/H/P reading without BSEC subscription
│   ├── Fixed 3-second reading interval
│   ├── Heater disabled (T/H/P only, no gas/IAQ for now)
│   ├── NVS state management (for future IAQ calibration)
│   └── **Future:** IAQ/CO2/VOC support requires BSEC subscription setup
│
└── can_protocol.c/h (~160 lines) - CAN message formatting
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

The firmware supports BME680/BME688 environmental sensors with **runtime auto-detection** and BSEC 2.6.1.0 library integration.

**Current Implementation (Simplified Mode):**
- **Temperature/Humidity/Pressure ONLY** - Direct sensor reading without BSEC virtual sensor subscription
- **No IAQ/CO2/VOC** - Requires BSEC subscription which is currently disabled
- **Fixed 3-second reading interval** - Simple periodic polling
- **Heater disabled** - Gas resistance not measured (saves power, no IAQ anyway)
- **Sensor configuration:** T_os=8x, H_os=2x, P_os=4x (high accuracy)

**Why Simplified Mode?**

The BSEC library has two modes of operation:
1. **Virtual sensor subscription** (BSEC-driven) - For IAQ/CO2/VOC outputs with dynamic timing
2. **Direct physical sensor reading** (Raw mode) - For basic T/H/P only

We use simplified mode because:
- The IAQ config blob (`bsec_config_iaq`) does NOT support requesting RAW outputs as virtual sensors
- Virtual sensor subscription is complex and requires specific output combinations
- For basic environmental monitoring, direct T/H/P reading is sufficient and more reliable

**Key Technical Details:**

**I2C Addresses:**
- Primary: 0x77 (default for most breakout boards)
- Alternate: 0x76 (configurable via SDO pin)
- Auto-detection tries both addresses

**Reading Flow** [bme680_bsec.c:492-719]:
1. Configure sensor with fixed settings (no BSEC control)
2. Trigger forced mode measurement
3. Wait ~82ms for measurement completion
4. Read raw T/H/P values from sensor
5. Feed data to BSEC in background (for future calibration)
6. Use raw sensor values directly (bypass BSEC outputs)

**Data Accuracy:**
- Temperature: ±1.0°C (typ), ±1.5°C (max)
- Humidity: ±3% RH (typ)
- Pressure: ±1.0 hPa (typ)

**NVS State Management:**
- Namespace: `bme68x_state`
- Saves BSEC calibration state every 4 hours (future IAQ use)
- Persists across reboots for faster IAQ convergence

**Future IAQ/CO2/VOC Support:**

To enable air quality outputs, the following changes are needed:

1. **Enable heater:**
   ```c
   heatr_conf.enable = BME68X_ENABLE;
   heatr_conf.heatr_temp = 320;  // °C (typical for IAQ)
   heatr_conf.heatr_dur = 150;   // ms
   ```

2. **Setup BSEC subscription:**
   ```c
   bsec_sensor_configuration_t requested_virtual_sensors[] = {
       {BSEC_OUTPUT_STATIC_IAQ, BSEC_SAMPLE_RATE_LP},
       // Note: Must use outputs supported by loaded config blob
       // Check BSEC_OUTPUT_INCLUDED bitfield (0x1279EF for current config)
   };
   ```

3. **Use BSEC-driven timing:**
   - Call `bsec_sensor_control()` before each measurement
   - Apply BSEC-provided sensor settings (not fixed settings)
   - Read at intervals specified by BSEC (typically 3s for LP mode)

4. **Wait for calibration:**
   - IAQ accuracy starts at 0 (uncalibrated)
   - Takes ~5 minutes to reach accuracy 1
   - Takes ~30 minutes to reach accuracy 3 (fully calibrated)
   - Calibration persists in NVS across reboots

**Known Issues:**
- BSEC subscription currently fails with warning 10 (incompatible output combination)
- Need to determine which virtual sensor outputs the loaded config actually supports
- May need different BSEC config blob for IAQ support

### CAN Protocol

**Message IDs:**
- `0x0A0`: RX - Master STOP command
- `0x0A1`: RX - Master START command
- `0x0A2`: TX - Lux data (1 Hz, normal operation)
- `0x0A3`: TX - Calibration commands (unused in current build)
- `0x0A4`: TX - Calibration responses (unused in current build)
- `0x0A5`: TX - Raw sensor data (unused in current build)

**Lux Data Format (0x0A2):**
```
Byte 0-2: Lux value (uint24_t, little-endian, 0-16,777,215)
Byte 3:   Status (0x00=OK, 0x01=Error)
Byte 4:   Sequence counter (rolls over)
Byte 5:   Current config index (0-20 for VEML7700, 100-111 for OPT4001)
Byte 6-7: Checksum (sum of bytes 0-5, uint16_t LE)
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

**Current Simplified Mode (T/H/P only):**

1. **Sensor configuration is FIXED** - not driven by BSEC
   - T_os=8x, H_os=2x, P_os=4x (high accuracy)
   - Heater disabled (no gas measurement)
   - Forced mode with fixed 3-second interval

2. **Values come from raw sensor** - not BSEC outputs
   - Direct reading from `bme68x_data` structure
   - BSEC processing runs in background but outputs are not used
   - This is intentional to avoid broken BSEC subscription

3. **Testing:** Verify T/H/P accuracy
   - Temperature should track room temperature (±1.5°C)
   - Humidity should be reasonable (20-80% indoors)
   - Pressure should match local barometric pressure (~980-1030 hPa)

**Future IAQ Mode (when enabling air quality):**

1. **BSEC subscription must be fixed first**
   - Current issue: Warning 10 (incompatible output combination)
   - Need to find which virtual sensors the config blob actually supports
   - May need different BSEC config blob

2. **Enable heater carefully**
   - Temperature: 320°C typical for IAQ
   - Duration: 150ms per measurement
   - Increases power consumption significantly
   - Adds self-heating offset to temperature reading

3. **BSEC timing is CRITICAL**
   - Must call `bsec_sensor_control()` before EVERY measurement
   - Must apply BSEC-provided settings (not fixed settings)
   - Must respect BSEC next_call timing (error 100 if called too early/late)

4. **Calibration takes time**
   - IAQ accuracy 0 = uncalibrated (first 5 minutes)
   - IAQ accuracy 1 = low accuracy (5-30 minutes)
   - IAQ accuracy 2 = medium accuracy (30+ minutes)
   - IAQ accuracy 3 = high accuracy (hours of operation)
   - State persists in NVS, so don't clear NVS during testing

5. **Stack size is critical**
   - BME680 task requires 10KB stack minimum
   - BSEC library uses significant stack space
   - Stack overflow will cause silent crashes

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
- Increase `SETTLE_READINGS_REQUIRED` [main.c:59]
- Increase moving average samples [main.c:63]

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

## Testing Checklist

### Unit Testing
- [ ] I2C communication (read/write registers)
- [ ] Sensor auto-detection (VEML7700 @ 0x10, OPT4001 @ 0x44)
- [ ] All 21 VEML7700 configurations apply correctly
- [ ] OPT4001 device ID verification (0x121)
- [ ] OPT4001 config register write/readback
- [ ] Checksum calculation for CAN messages
- [ ] Moving average filter output (VEML7700)
- [ ] 3-byte lux encoding/decoding

### Integration Testing - VEML7700
- [ ] Auto-ranging across 0-120,000 lux
- [ ] Saturation ramp-up in bright light (watch for rapid 8→9→...→20 progression)
- [ ] No oscillation between configs
- [ ] Smooth transitions: indoor (500 lux) → outdoor (70K lux)
- [ ] Calibration accuracy vs reference meter (indoor and sunlight)
- [ ] Config index 0-20 reported correctly in CAN messages

### Integration Testing - OPT4001
- [ ] Hardware auto-ranging across 0-2.2M lux
- [ ] Accuracy vs reference meter: ±2% at 100 lux, 1K lux, 10K+ lux
- [ ] Range transitions smooth (config 100-111)
- [ ] Config index 100-111 reported correctly in CAN messages
- [ ] No stack overflow with powf() usage

### Integration Testing - Common
- [ ] START/STOP commands work
- [ ] Sensor type correctly identified from config index

### Reliability Testing
- [ ] 24-hour continuous operation
- [ ] No memory leaks (check heap)
- [ ] Handles sensor disconnect gracefully
- [ ] CAN bus error recovery

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
#define CAN_DATA_PERIOD_MS      1000    // CAN transmission rate [main.c:26]
#define SENSOR_QUEUE_DEPTH      20      // Sensor data queue size [main.c:27]
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

**Next Phases (Planned):**
- Phase 1: Add BME680 environmental sensor with BSEC 2.x
- Phase 2: Add HLK-LD2410 human presence radar (UART)
- Phase 3: Add BME688 air quality sensor (enhanced BSEC)
- Phase 4: Add MQ-3 alcohol/VOC gas sensor (ADC)

See `EXPANSION_GUIDE.md` for detailed implementation plans including:
- Hardware pin assignments
- Resource budgets (SRAM, Flash, CPU)
- Extended CAN protocol (0x0A3-0x0A7)
- Task coordination strategies
- Testing procedures
