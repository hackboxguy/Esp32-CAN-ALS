# BME680/BME688 Environmental Sensor Setup Guide

## Overview

This firmware supports **optional** BME680/BME688 environmental sensors using Bosch's proprietary **BSEC 2.x library** for accurate air quality measurements.

**Measurements provided:**
- Temperature (°C)
- Humidity (%RH)
- Barometric pressure (hPa)
- Indoor Air Quality Index (IAQ, 0-500)
- CO₂ equivalent (ppm)
- Breath VOC equivalent (ppm)

## Why is BSEC Library Separate?

The BSEC library is **proprietary software** from Bosch Sensortec and cannot be included in this open-source repository. Users must:
1. Download it directly from Bosch
2. Accept Bosch's license agreement
3. Run a setup script to integrate it

**If you don't need BME680 support**, simply skip this setup. The firmware will build without it and only support ambient light sensors (VEML7700/OPT4001).

---

## Quick Setup (3 Steps)

### Step 1: Download BSEC Library

**Option A - Official Bosch Website (Recommended):**
1. Visit: https://www.bosch-sensortec.com/software-tools/software/bme680-software-bsec/
2. Click "Download" and accept the license agreement
3. Download the **Generic Release** (e.g., `bsec2-6-1-0_generic_release_22102024.zip`)

**Option B - GitHub:**
1. Visit: https://github.com/boschsensortec/Bosch-BSEC2-Library/releases
2. Download the latest release zip

### Step 2: Run Setup Script

```bash
cd components/bsec
./setup_bsec.sh ~/Downloads/bsec2-6-1-0_generic_release_22102024.zip
```

**Expected output:**
```
=================================================
  BSEC Library Setup for ESP32-C3/C6
=================================================

✓ Found BSEC zip: ...
✓ Extracted BSEC library
✓ Copied BSEC headers (bsec_*.h)
✓ Copied libalgobsec.a (176K)
✓ Copied BME68x sensor API

=================================================
  BSEC Setup Complete!
=================================================
```

### Step 3: Enable in Build Configuration

```bash
idf.py menuconfig
```

Navigate to:
```
Component config
  → BSEC Library Configuration
    → [*] Enable BME680/BME688 support (requires BSEC library)
```

Save and exit (press `S`, then `Q`).

### Step 4: Build and Flash

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Hardware Setup

### BME680/BME688 Wiring

Connect BME680/BME688 to ESP32 via I2C:

```
BME680/688    ESP32-C3/C6
─────────────────────────
VCC      →    3.3V
GND      →    GND
SDA      →    GPIO6  (I2C SDA, shared with VEML7700/OPT4001)
SCL      →    GPIO7  (I2C SCL, shared with VEML7700/OPT4001)
SDO      →    GND    (sets I2C address to 0x76)
```

**I2C Address Configuration:**
- SDO = GND → Address **0x76** (default in firmware)
- SDO = VCC → Address **0x77** (change in menuconfig)

**Important:** BME680 and ambient light sensor (VEML7700/OPT4001) share the I2C bus. Both can be connected simultaneously since they have different addresses.

### Multi-Sensor Configuration

| Sensor | I2C Address | Function |
|--------|-------------|----------|
| VEML7700 | 0x10 | Ambient Light (0-120K lux) |
| OPT4001 | 0x44 | Ambient Light (0-2.2M lux) |
| BME680 | 0x76 | Environment + Air Quality |

**Valid combinations:**
- ✅ VEML7700 + BME680
- ✅ OPT4001 + BME680
- ❌ VEML7700 + OPT4001 (only one ALS at a time)

---

## Build Configuration Options

All options available in `idf.py menuconfig` under **BSEC Library Configuration**:

### Enable BME680/BME688 Support
**Default:** Disabled
**Description:** Master enable switch for BME680 functionality

### BME680 I2C Address
**Default:** 0x76
**Range:** 0x76 or 0x77
**Description:** Hardware I2C address (set by SDO pin)

### BME680 Sample Rate
**Default:** 3 seconds
**Range:** 1-60 seconds
**Description:** How often to read the sensor. BSEC requires regular sampling for accurate IAQ calculation.

**Recommendations:**
- **3 seconds** - Best balance of accuracy and power
- **1 second** - Fastest response, higher CPU load
- **>5 seconds** - May reduce IAQ accuracy

### BSEC State Save Interval
**Default:** 4 hours
**Range:** 1-24 hours
**Description:** How often to save BSEC calibration state to NVS (non-volatile storage)

**Why this matters:**
BSEC learns air quality baselines over time (up to 28 days). Saving state preserves calibration across reboots.

**Recommendations:**
- **4 hours** - Default, balances flash wear with state freshness
- **1 hour** - More frequent saves, more flash writes
- **24 hours** - Minimal flash wear, but lose more calibration on power loss

---

## Verifying Installation

### Check Component Detection

```bash
idf.py reconfigure 2>&1 | grep BSEC
```

**Expected output (BSEC installed):**
```
-- BSEC library found - BME680 support available
-- BSEC library linked: .../components/bsec/lib/libalgobsec.a
```

**Expected output (BSEC NOT installed):**
```
  BSEC library NOT found
  BME680 support DISABLED
```

### Check Binary Size

After building, check the BSEC library impact:

```bash
idf.py size
```

**BSEC overhead:**
- Flash: ~175 KB (precompiled library)
- RAM: ~40 KB (BSEC algorithm state)

### Check Runtime Logs

After flashing, monitor serial output:

```bash
idf.py monitor
```

**Expected logs (BME680 enabled and detected):**
```
I (XXX) MULTI_SENSOR: BME680 initialized at I2C address 0x76
I (XXX) BSEC: BSEC version 2.6.1.0
I (XXX) BSEC: Loaded calibration state from NVS (age: 2 hours)
```

**Expected logs (BME680 not connected):**
```
W (XXX) MULTI_SENSOR: BME680 not detected on I2C bus
I (XXX) MULTI_SENSOR: Continuing without BME680 support
```

---

## CAN Bus Integration

When BME680 is enabled, the firmware transmits environmental data via CAN:

### Message IDs

| CAN ID | Data Type | Rate |
|--------|-----------|------|
| 0x100 | Ambient Light (VEML7700/OPT3001/OPT4001) | 1 Hz |
| 0x101 | Environmental (Temp/Humid/Press) | 0.33 Hz |
| 0x102 | Air Quality (IAQ/CO2/VOC) | 0.33 Hz |

### Message Format - Environmental Data (0x101)

```
Byte 0-1: Temperature × 100 (int16_t, °C, little-endian)
Byte 2-3: Humidity × 100 (uint16_t, %RH, LE)
Byte 4-5: Pressure (uint16_t, hPa, LE)
Byte 6:   Status (0x00=OK, 0x01=Error)
Byte 7:   Checksum (sum of bytes 0-6)
```

**Example decoding:**
- Bytes [0x10, 0x09] → 0x0910 = 2320 → 23.20°C
- Bytes [0x88, 0x13] → 0x1388 = 5000 → 50.00%RH
- Bytes [0xE8, 0x03] → 0x03E8 = 1000 → 1000 hPa

### Message Format - Air Quality (0x102)

```
Byte 0-1: IAQ (uint16_t, 0-500, little-endian)
Byte 2:   Accuracy (0-3, BSEC calibration status)
Byte 3-4: CO₂ equivalent (uint16_t, ppm, LE)
Byte 5-6: Breath VOC (uint16_t, ppm, LE)
Byte 7:   Checksum (sum of bytes 0-6)
```

**IAQ Interpretation:**
- 0-50: Excellent air quality
- 51-100: Good
- 101-150: Lightly polluted
- 151-200: Moderately polluted
- 201-250: Heavily polluted
- 251-350: Severely polluted
- 351-500: Extremely polluted

**Accuracy Interpretation:**
- 0: Sensor stabilizing (first 5 minutes)
- 1: Low accuracy (calibrating)
- 2: Medium accuracy (partial calibration)
- 3: High accuracy (fully calibrated)

---

## Troubleshooting

### Build Issues

**Error: "BSEC library not found"**
```
Solution: Run setup script
$ cd components/bsec
$ ./setup_bsec.sh /path/to/bsec_zip_file.zip
```

**Error: "undefined reference to 'bsec_do_steps'"**
```
Solution: Check menuconfig has BME680 enabled
$ idf.py menuconfig → BSEC Library Configuration
```

**Error: "file format not recognized"**
```
Solution: Wrong target chip variant
$ idf.py set-target esp32c3  # or esp32c6
$ idf.py fullclean && idf.py build
```

### Runtime Issues

**BME680 not detected on I2C bus**
- Check wiring (SDA/SCL not swapped)
- Verify 3.3V power supply
- Check I2C address (SDO pin determines 0x76 vs 0x77)
- Try: `idf.py menuconfig` → Change BME680 I2C Address

**IAQ always shows 0 or 25**
- Sensor needs calibration (wait 5-30 minutes)
- Check accuracy byte in CAN message (should reach 2-3)
- BSEC learns baselines over time (up to 28 days for full accuracy)

**Temperature readings incorrect**
- BME680 self-heats (~3-5°C above ambient)
- Apply offset correction in post-processing
- Position sensor away from heat sources

**No CAN messages for BME680**
- Check logs: `idf.py monitor`
- Verify BME680 enabled in menuconfig
- Check CAN bus wiring and termination
- Monitor with: `candump can0 | grep 0A3`

---

## Resource Usage

### Flash Memory
- BSEC library: ~175 KB
- BME68x driver: ~5 KB
- Integration code: ~3 KB
- **Total overhead: ~183 KB**

### RAM Usage
- BSEC algorithm state: ~40 KB
- Task stack (bsec_task): 10 KB
- Sensor buffers: ~2 KB
- **Total overhead: ~52 KB**

### CPU Usage
- BSEC algorithm: ~2-3% @ 160 MHz
- I2C communication: <1%
- **Total overhead: ~3-4%**

---

## Uninstalling BSEC

To remove BSEC support and free up flash/RAM:

### Option 1: Disable in menuconfig (keeps library installed)
```bash
idf.py menuconfig
→ BSEC Library Configuration
→ [ ] Enable BME680/BME688 support
```

### Option 2: Remove library files (complete removal)
```bash
cd components/bsec
rm -rf lib/ include/ src/
idf.py fullclean
idf.py build
```

The firmware will automatically detect BSEC is missing and build without BME680 support.

---

## Advanced Configuration

### Custom BSEC Configuration

BSEC supports different configuration profiles optimized for different scenarios. To use a custom config:

1. Download config files from Bosch (included in BSEC zip)
2. Edit `bme680_bsec.c` (when created in future commits)
3. Replace `generic_33v_3s_4d` config with desired profile

**Available profiles:**
- `generic_33v_3s_4d` - General purpose (3s sample, 4 days calibration)
- `generic_33v_300s_4d` - Low power (5min sample)
- Custom profiles can be generated with Bosch tools

### NVS State Management

BSEC calibration state is stored in NVS namespace `bsec_state`:

**View stored state:**
```bash
idf.py monitor
(press Ctrl+]) to enter monitor
nvs_get bsec_state calibration
```

**Erase stored state (force recalibration):**
```bash
idf.py erase-flash
idf.py flash
```

---

## References

- **BSEC Documentation:** https://www.bosch-sensortec.com/software-tools/software/bme680-software-bsec/
- **BME680 Datasheet:** https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/
- **ESP-IDF I2C Guide:** https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/i2c.html

---

## License

**BSEC Library:** Proprietary license from Bosch Sensortec
**This integration code:** MIT License (see repository [LICENSE](LICENSE))
