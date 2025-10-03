# BSEC Library Setup (Optional Component)

This directory contains the optional **Bosch BSEC 2.x** library integration for BME680/BME688 environmental sensors.

## Why is BSEC not included?

The BSEC library is **proprietary software** from Bosch Sensortec and cannot be redistributed in this open-source repository. Users must download it directly from Bosch and accept their license agreement.

## Setup Instructions

### Step 1: Download BSEC Library

Download the latest BSEC 2.x library from Bosch Sensortec:

**Option A - Official Website (Recommended):**
1. Visit: https://www.bosch-sensortec.com/software-tools/software/bme680-software-bsec/
2. Accept the license agreement
3. Download the **Generic Release** zip file (e.g., `bsec2-6-1-0_generic_release_22102024.zip`)

**Option B - GitHub:**
1. Visit: https://github.com/boschsensortec/Bosch-BSEC2-Library/releases
2. Download the latest release

### Step 2: Extract BSEC Library

Run the setup script with the path to your downloaded zip file:

```bash
cd components/bsec
./setup_bsec.sh /path/to/bsec2-6-1-0_generic_release_22102024.zip
```

The script will:
- Extract the necessary files
- Copy the ESP32-C3/C6 precompiled library
- Copy required header files
- Copy BME68x sensor API files

### Step 3: Enable BME680 Support

Configure the build to enable BME680 support:

```bash
idf.py menuconfig
```

Navigate to: **Component config → Multi-Sensor CAN Node → BME680 Environmental Sensor**

Enable: `[*] Enable BME680/BME688 support (requires BSEC library)`

### Step 4: Build

```bash
idf.py build
```

## Building WITHOUT BME680 Support

If you don't need BME680 functionality, simply **skip the setup steps above**. The firmware will build without BME680 support and only include the ambient light sensors (VEML7700/OPT4001).

The build system automatically detects if BSEC is available:
- ✅ **BSEC found** → BME680 support enabled (if Kconfig enabled)
- ❌ **BSEC not found** → BME680 support disabled (graceful fallback)

## Directory Structure (After Setup)

```
components/bsec/
├── README.md                  # This file
├── setup_bsec.sh             # Setup script
├── CMakeLists.txt            # ESP-IDF component build config
├── Kconfig                   # Configuration options
├── include/
│   ├── bsec_datatypes.h     # BSEC data types
│   ├── bsec_interface.h     # BSEC API
│   ├── bme68x.h             # BME68x sensor API
│   └── bme68x_defs.h        # BME68x definitions
├── src/
│   └── bme68x.c             # BME68x sensor driver
└── lib/
    └── libalgobsec.a        # Precompiled BSEC library (ESP32-C3/C6)
```

## Verifying Installation

After running the setup script, check that these files exist:

```bash
ls -l components/bsec/lib/libalgobsec.a
ls -l components/bsec/include/bsec_interface.h
ls -l components/bsec/src/bme68x.c
```

## License

**BSEC Library:** Proprietary license from Bosch Sensortec (see files extracted from zip)
**This integration code:** MIT License (see repository LICENSE)

## Troubleshooting

**Error: "BSEC library not found"**
- Run the setup script: `./setup_bsec.sh /path/to/bsec.zip`

**Error: "libalgobsec.a: file format not recognized"**
- Wrong BSEC binary variant. Ensure you're building for ESP32-C3 or ESP32-C6
- Run: `idf.py set-target esp32c3` or `idf.py set-target esp32c6`

**Build succeeds but no BME680 data on CAN bus**
- Check Kconfig: BME680 support must be enabled in menuconfig
- Verify I2C wiring: BME680 address 0x76 or 0x77
- Check logs: `idf.py monitor` should show "BME680 initialized"

## Technical Details

**Supported ESP32 Variants:**
- ESP32-C3 (RISC-V, 160 MHz)
- ESP32-C6 (RISC-V, 160 MHz)

**BSEC Library Version:**
- BSEC 2.6.1.0 or later

**BSEC Binary Used:**
- Path: `algo/bsec_IAQ/bin/esp/esp32_c2c3/libalgobsec.a`
- Architecture: RISC-V 32-bit (RV32IMC)
- Size: ~175 KB (Flash), ~40 KB (RAM)

**BME68x Sensor API:**
- Version: Latest from BSEC distribution
- Source: `examples/BSEC_Integration_Examples/src/bme68x/`
