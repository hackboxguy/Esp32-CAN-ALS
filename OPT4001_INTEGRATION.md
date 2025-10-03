# OPT4001 Integration Guide

## Summary

Added support for OPT4001 ambient light sensor with automatic runtime detection. The firmware now supports either VEML7700 or OPT4001, automatically detecting which sensor is present on the I2C bus at startup.

## Key Features

✅ **Runtime Auto-Detection**
- Probes I2C bus at startup for VEML7700 (0x10) or OPT4001 (0x44)
- Single firmware binary works with either sensor
- No recompilation needed when swapping sensors

✅ **Unified API**
- Clean abstraction layer (als_driver.c/h)
- Same CAN message format (0x0A2) for both sensors
- Backward compatible with existing VEML7700 deployments

✅ **Sensor-Specific Handling**
- VEML7700: Manual auto-ranging + calibration correction (0-120K lux)
- OPT4001: Hardware auto-ranging, factory calibrated (0-2.2M lux)

## File Structure

### New Files Created
- `main/opt4001_driver.c/h` (~200 lines) - OPT4001 sensor driver
- `main/als_driver.c/h` (~150 lines) - Unified abstraction layer

### Modified Files
- `main/main.c` - Changed from `veml7700_*` to `als_*` API
- `main/CMakeLists.txt` - Added new source files

### Unchanged Files
- `main/veml7700_driver.c/h` - Preserved for VEML7700 support
- `main/can_protocol.c/h` - No changes needed
- `main/sensor_common.h` - No changes needed

## Config Index Convention

To allow CAN clients to distinguish sensor types from the config field:

**VEML7700:**
- Config index: **0-20**
- Represents manual configuration (gain × integration time)

**OPT4001:**
- Config index: **100-111**
- Represents hardware auto-range index (0-11) + 100 offset

**Example CAN Message Decoding:**
```
Byte 5 = 0-20   → VEML7700 active
Byte 5 = 100-111 → OPT4001 active
```

## OPT4001 vs VEML7700 Comparison

| Feature | VEML7700 | OPT4001 |
|---------|----------|---------|
| **I2C Address** | 0x10 | 0x44 |
| **Range** | 0-120K lux | 0-2.2M lux |
| **Auto-Ranging** | Manual (firmware) | Hardware (automatic) |
| **Configurations** | 21 (manual switching) | 12 (auto-switching) |
| **Calibration** | Required (diffuser correction) | Factory calibrated |
| **Code Complexity** | ~550 lines | ~200 lines |
| **Settling Time** | 800ms + config switches | 800ms (no config changes) |
| **Accuracy** | ±1.4% (with calibration) | ±2% (factory) |

## How It Works

### Startup Sequence

```
1. als_init() called from main.c
2. Probe I2C bus for 0x10 (VEML7700)
   └─ If found: Initialize VEML7700, set detected_sensor = VEML7700
3. If not found, probe 0x44 (OPT4001)
   └─ If found: Initialize OPT4001, set detected_sensor = OPT4001
4. If neither found: Return ESP_ERR_NOT_FOUND
```

### Runtime Operation

```
sensor_poll_task (1 Hz):
  ├─ Call als_read_lux()
  │   ├─ If VEML7700: veml7700_read_lux() + calibration
  │   └─ If OPT4001:  opt4001_read_lux() (no calibration)
  ├─ Call als_get_config_idx()
  │   ├─ If VEML7700: Return 0-20
  │   └─ If OPT4001:  Return 100 + range_index
  └─ Send via CAN (same format)
```

## OPT4001 Technical Details

### Register Map
- **0x00:** Result register (lux data, 32-bit)
- **0x01:** Configuration register
- **0x02:** Flags register
- **0x11:** Device ID (0x1121)

### Configuration
```c
// Settings used in opt4001_init()
- Auto-range mode enabled
- 800ms conversion time (highest accuracy)
- Continuous conversion mode
- Interrupt latching enabled
```

### Lux Calculation
```c
// OPT4001 uses mantissa + exponent format
exponent = (result >> 20) & 0x0F;    // Bits 20-23
mantissa = result & 0xFFFFF;         // Bits 0-19

lux = mantissa * 2^exponent * 0.01;
```

### Auto-Range Levels (Hardware Managed)
- Range 0: 0.001 - 5.24 lux
- Range 1: 0.002 - 10.5 lux
- Range 2: 0.004 - 21 lux
- ...
- Range 11: 2621 - 2,200,000 lux

## Testing

### Build Firmware
```bash
idf.py build
```

### Expected Build Output
```
[x/y] Building C object...
- opt4001_driver.c
- als_driver.c
...
[100%] Built target esp32_can_counter.elf
```

### Flash and Monitor
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

### Expected Startup Logs

**With VEML7700:**
```
I (xxx) ALS: Auto-detecting ambient light sensor...
I (xxx) ALS: VEML7700 detected at address 0x10
I (xxx) VEML7700: Config 8: 1x gain, 100ms IT
I (xxx) VEML7700: Initialized successfully
I (xxx) ALS: Using VEML7700 (0-120K lux, with calibration)
I (xxx) MULTI_SENSOR: Ambient light sensor (VEML7700) initialized successfully
```

**With OPT4001:**
```
I (xxx) ALS: Auto-detecting ambient light sensor...
I (xxx) ALS: OPT4001 detected at address 0x44
I (xxx) OPT4001: Device ID verified: 0x1121
I (xxx) OPT4001: Config written=0x3CC0, read=0x3CC0
I (xxx) OPT4001: Initialized successfully (auto-range, 800ms, continuous)
I (xxx) ALS: Using OPT4001 (0-2.2M lux, factory calibrated)
I (xxx) MULTI_SENSOR: Ambient light sensor (OPT4001) initialized successfully
```

### Expected CAN Messages

**VEML7700 (Indoor ~1000 lux):**
```
can0  0A2   [8]  E8 03 00 00 XX 08 ...
                  ^-----^     ^
                  1000 lux    Config 8 (VEML7700)
```

**OPT4001 (Indoor ~1000 lux):**
```
can0  0A2   [8]  E8 03 00 00 XX 67 ...
                  ^-----^     ^
                  1000 lux    Config 103 (OPT4001, range 3)
```

**OPT4001 (Sunlight ~100K lux):**
```
can0  0A2   [8]  A0 86 01 00 XX 6A ...
                  ^--------^     ^
                  100000 lux     Config 106 (OPT4001, range 6)
```

## Troubleshooting

### No Sensor Detected
```
E (xxx) ALS: No ambient light sensor detected!
E (xxx) ALS: Checked: 0x10 (VEML7700), 0x44 (OPT4001)
E (xxx) MULTI_SENSOR: Failed to initialize ambient light sensor
```

**Solutions:**
- Check I2C wiring (SDA=GPIO6, SCL=GPIO7)
- Verify sensor power (3.3V)
- Check I2C pull-ups (4.7kΩ recommended)
- Use `i2cdetect` to scan I2C bus: `i2cdetect -y 0`

### OPT4001 Wrong Device ID
```
E (xxx) OPT4001: Invalid device ID: 0xXXXX (expected 0x1121)
```

**Solutions:**
- Verify OPT4001 hardware (not OPT3001/OPT3002)
- Check I2C communication integrity
- Try slower I2C clock (currently 100kHz)

### CAN Client Shows Wrong Config Index
- If config > 20: OPT4001 is active (subtract 100 to get range index)
- If config 0-20: VEML7700 is active

## Advantages of OPT4001

1. **Wider Range:** 2.2M lux vs 120K lux (18x greater)
2. **Simpler Code:** Hardware auto-ranging (no manual config switching)
3. **No Calibration Needed:** Factory calibrated, no diffuser correction
4. **Faster Response:** No settling delays during ranging
5. **Better Linearity:** Superior optical design

## Migration Notes

### From VEML7700-only to Dual Support

**Code Changes Required:**
- ✅ Include `als_driver.h` instead of `veml7700_driver.h`
- ✅ Change `veml7700_init()` → `als_init()`
- ✅ Change `veml7700_read_lux()` → `als_read_lux()`
- ✅ Change `veml7700_get_config_idx()` → `als_get_config_idx()`

**Backward Compatibility:**
- Existing VEML7700 hardware works unchanged
- Same CAN message format
- VEML7700 calibration still active
- Config indices 0-20 preserved for VEML7700

### Adding OPT4001 to Existing Deployment

1. Replace VEML7700 module with OPT4001 on PCB
2. Reflash firmware (no code changes needed)
3. Firmware auto-detects OPT4001
4. CAN messages use config index 100-111 (clients can distinguish)

## Future Enhancements

Potential improvements:

- [ ] Add NVS storage for sensor type preference
- [ ] Implement sensor health monitoring (detect disconnects)
- [ ] Add support for OPT3001/OPT3002 (compatible protocol)
- [ ] Expose sensor type via CAN status message (0x0A7)
- [ ] Add compile-time sensor selection for size optimization

## Files Modified Summary

```
main/
├── opt4001_driver.c (NEW, 200 lines)
├── opt4001_driver.h (NEW, 50 lines)
├── als_driver.c (NEW, 150 lines)
├── als_driver.h (NEW, 70 lines)
├── main.c (MODIFIED, 3 function calls updated)
├── CMakeLists.txt (MODIFIED, added 2 source files)
├── veml7700_driver.c (UNCHANGED)
└── veml7700_driver.h (UNCHANGED)
```

**Total Code Added:** ~470 lines
**Code Removed:** 0 lines (fully backward compatible)

## Ready to Test!

The implementation is complete. To test:

1. **Build:** `idf.py build`
2. **Flash:** `idf.py -p /dev/ttyUSB0 flash monitor`
3. **Verify:** Check startup logs to see which sensor was detected
4. **Test CAN:** `candump can0` - Check config index (0-20 or 100-111)

Both VEML7700 and OPT4001 should work seamlessly with the same firmware!
