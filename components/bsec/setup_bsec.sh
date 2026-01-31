#!/bin/bash
# BSEC Library Setup Script for ESP32-C3/C6
# Usage: ./setup_bsec.sh <path_to_bsec_zip>

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BSEC_ZIP="$1"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "================================================="
echo "  BSEC Library Setup for ESP32-C3/C6"
echo "================================================="
echo ""

# Check if zip file argument provided
if [ -z "$BSEC_ZIP" ]; then
    echo -e "${RED}Error: No BSEC zip file specified${NC}"
    echo ""
    echo "Usage: $0 <path_to_bsec_zip>"
    echo ""
    echo "Example:"
    echo "  $0 ~/Downloads/bsec2-6-1-0_generic_release_22102024.zip"
    echo ""
    echo "Download BSEC from:"
    echo "  https://www.bosch-sensortec.com/software-tools/software/bme680-software-bsec/"
    exit 1
fi

# Check if zip file exists
if [ ! -f "$BSEC_ZIP" ]; then
    echo -e "${RED}Error: File not found: $BSEC_ZIP${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} Found BSEC zip: $BSEC_ZIP"
echo ""

# Create temporary extraction directory
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR" EXIT

echo "Extracting BSEC library..."
unzip -q "$BSEC_ZIP" -d "$TEMP_DIR"

# Find the extracted directory (handle different archive structures)
BSEC_ROOT=$(find "$TEMP_DIR" -type d -name "algo" -exec dirname {} \; | head -n 1)

if [ -z "$BSEC_ROOT" ]; then
    echo -e "${RED}Error: Invalid BSEC archive structure${NC}"
    echo "Expected to find 'algo' directory in zip file"
    exit 1
fi

echo -e "${GREEN}✓${NC} Extracted BSEC library"

# Create directory structure
echo "Creating component directory structure..."
mkdir -p "$SCRIPT_DIR/include"
mkdir -p "$SCRIPT_DIR/src"
mkdir -p "$SCRIPT_DIR/lib"

# Copy BSEC headers
echo "Copying BSEC header files..."
if [ -d "$BSEC_ROOT/algo/bsec_IAQ/inc" ]; then
    cp "$BSEC_ROOT/algo/bsec_IAQ/inc/"*.h "$SCRIPT_DIR/include/"
    echo -e "${GREEN}✓${NC} Copied BSEC headers (bsec_*.h)"
else
    echo -e "${RED}Error: BSEC headers not found${NC}"
    exit 1
fi

# Copy BSEC precompiled library for ESP32-C3/C6
echo "Copying BSEC precompiled library (ESP32-C2/C3)..."
BSEC_LIB="$BSEC_ROOT/algo/bsec_IAQ/bin/esp/esp32_c2c3/libalgobsec.a"
if [ -f "$BSEC_LIB" ]; then
    cp "$BSEC_LIB" "$SCRIPT_DIR/lib/"
    echo -e "${GREEN}✓${NC} Copied libalgobsec.a ($(du -h "$BSEC_LIB" | cut -f1))"
else
    echo -e "${RED}Error: ESP32-C3 library not found at: $BSEC_LIB${NC}"
    exit 1
fi

# Copy BME68x sensor API files
echo "Copying BME68x sensor API..."
BME68X_SRC="$BSEC_ROOT/examples/BSEC_Integration_Examples/src/bme68x"
if [ -d "$BME68X_SRC" ]; then
    cp "$BME68X_SRC/bme68x.c" "$SCRIPT_DIR/src/"
    cp "$BME68X_SRC/bme68x.h" "$SCRIPT_DIR/include/"
    cp "$BME68X_SRC/bme68x_defs.h" "$SCRIPT_DIR/include/"
    echo -e "${GREEN}✓${NC} Copied BME68x sensor API (bme68x.c, bme68x.h, bme68x_defs.h)"
else
    echo -e "${RED}Error: BME68x sensor API not found${NC}"
    exit 1
fi

# Verify all files are in place
echo ""
echo "Verifying installation..."
REQUIRED_FILES=(
    "include/bsec_datatypes.h"
    "include/bsec_interface.h"
    "include/bme68x.h"
    "include/bme68x_defs.h"
    "src/bme68x.c"
    "lib/libalgobsec.a"
)

ALL_FOUND=true
for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$SCRIPT_DIR/$file" ]; then
        echo -e "${GREEN}✓${NC} $file"
    else
        echo -e "${RED}✗${NC} $file (MISSING)"
        ALL_FOUND=false
    fi
done

echo ""
if [ "$ALL_FOUND" = true ]; then
    echo -e "${GREEN}=================================================${NC}"
    echo -e "${GREEN}  BSEC Setup Complete!${NC}"
    echo -e "${GREEN}=================================================${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Enable BME680 support:"
    echo "     $ idf.py menuconfig"
    echo "     → Component config → Multi-Sensor CAN Node"
    echo "     → [*] Enable BME680/BME688 support"
    echo ""
    echo "  2. Build the project:"
    echo "     $ idf.py build"
    echo ""
    echo "Library info:"
    echo "  - BSEC version: $(basename "$BSEC_ZIP" .zip)"
    echo "  - Target: ESP32-C3, ESP32-C6 (RISC-V)"
    echo "  - Binary size: $(du -h "$SCRIPT_DIR/lib/libalgobsec.a" | cut -f1)"
    echo ""
else
    echo -e "${RED}=================================================${NC}"
    echo -e "${RED}  Setup Failed - Missing Files${NC}"
    echo -e "${RED}=================================================${NC}"
    exit 1
fi
