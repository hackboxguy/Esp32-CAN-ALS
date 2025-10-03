# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Summary

ESP32-C3/C6 automotive CAN node with VEML7700 ambient light sensor. Production-ready implementation featuring intelligent auto-ranging, moving average filtering, and a CAN-based calibration system. Modular architecture with queue-based sensor data flow, designed for multi-sensor expansion.

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
├── veml7700_driver.c/h (~550 lines) - VEML7700 sensor module
│   ├── I2C communication functions
│   ├── Configuration table (21 configs: gain × integration time)
│   ├── Intelligent auto-ranging algorithm with saturation ramp-up
│   ├── Moving average filter (8 samples)
│   └── Calibration correction (piecewise linear, 0-120K lux range)
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
│   (Priority 5)  │  │
└─────────────────┘  │
                     │    ┌──────────────┐    ┌──────────────────┐
                     ├───→│ sensor_queue │───→│ twai_transmit    │──→ CAN Bus
                     │    │  (depth: 20) │    │    _task (P8)    │
┌─────────────────┐  │    └──────────────┘    └──────────────────┘
│ (future sensors)│──┘                                ▲
└─────────────────┘                                   │
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
Byte 5:   Current config index (0-20, for debugging)
Byte 6-7: Checksum (sum of bytes 0-5, uint16_t LE)
```

### Pin Assignments

```
GPIO4:  CAN TX
GPIO5:  CAN RX
GPIO6:  I2C SDA (VEML7700)
GPIO7:  I2C SCL (VEML7700)
```

### Resource Usage

- **Flash:** ~55-65 KB (modular build)
- **SRAM:** ~70 KB (out of 400 KB available)
- **CPU Load:** <5% @ 160 MHz
- **Task Stacks:**
  - sensor_poll_task: 2 KB
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

## Testing Checklist

### Unit Testing
- [ ] I2C communication (read/write registers)
- [ ] All 21 configurations apply correctly
- [ ] Checksum calculation for CAN messages
- [ ] Moving average filter output
- [ ] 3-byte lux encoding/decoding

### Integration Testing
- [ ] Auto-ranging across 0-120,000 lux
- [ ] Saturation ramp-up in bright light (watch for rapid 8→9→...→20 progression)
- [ ] No oscillation between configs
- [ ] START/STOP commands work
- [ ] Smooth transitions: indoor (500 lux) → outdoor (70K lux)
- [ ] Calibration accuracy vs reference meter (indoor and sunlight)

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
