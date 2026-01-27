#!/bin/bash
# build.sh - Build script for ESP32-CAN-Sensor firmware
#
# Usage:
#   ./build.sh --target=esp32c6
#   ./build.sh --target=esp32c3
#   ./build.sh --target=esp32c6 --idfpath=/opt/esp-idf
#   ./build.sh --clean
#   ./build.sh --flash --port=/dev/ttyUSB0
#   ./build.sh --flash-only --port=/dev/ttyUSB0

set -e

# Default values
TARGET=""
IDF_PATH_ARG=""
BSEC_PATH=""
CLEAN=false
FLASH=false
FLASH_ONLY=false
MONITOR=false
PORT=""
VERSION_MAJOR=""
VERSION_MINOR=""
VERSION_PATCH=""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --target=TARGET   ESP32 target: 'esp32c3' or 'esp32c6' (required for first build)"
    echo "  --idfpath=PATH    Path to ESP-IDF installation (default: ~/esp/esp-idf)"
    echo "  --bsecpath=PATH   Path to BSEC library zip (for BME680/688 support)"
    echo "  --port=PORT       Serial port for flashing (e.g., /dev/ttyUSB0)"
    echo "  --clean           Clean build directory before building"
    echo "  --flash           Flash firmware after building"
    echo "  --flash-only      Flash existing binary without rebuilding"
    echo "  --monitor         Start serial monitor after flashing"
    echo "  --version=X.Y.Z   Set firmware version (e.g., --version=1.2.3)"
    echo "  --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 --target=esp32c6"
    echo "  $0 --target=esp32c6 --flash --port=/dev/ttyUSB0"
    echo "  $0 --flash-only --port=/dev/ttyUSB0  # Flash without rebuild"
    echo "  $0 --target=esp32c3 --bsecpath=/path/to/bsec.zip"
    echo "  $0 --clean --version=2.1.0"
    echo ""
    echo "Build output:"
    echo "  build/esp32-can-sensor.bin"
}

# Parse arguments
for arg in "$@"; do
    case $arg in
        --target=*)
            TARGET="${arg#*=}"
            ;;
        --idfpath=*)
            IDF_PATH_ARG="${arg#*=}"
            ;;
        --bsecpath=*)
            BSEC_PATH="${arg#*=}"
            ;;
        --port=*)
            PORT="${arg#*=}"
            ;;
        --version=*)
            VERSION="${arg#*=}"
            IFS='.' read -r VERSION_MAJOR VERSION_MINOR VERSION_PATCH <<< "$VERSION"
            ;;
        --clean)
            CLEAN=true
            ;;
        --flash)
            FLASH=true
            ;;
        --flash-only)
            FLASH_ONLY=true
            ;;
        --monitor)
            MONITOR=true
            ;;
        --help)
            print_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Error: Unknown argument: $arg${NC}"
            print_usage
            exit 1
            ;;
    esac
done

# flash-only implies flash
if [[ "$FLASH_ONLY" == true ]]; then
    FLASH=true
fi

# Build directory
BUILD_DIR="build"

# Determine IDF path
if [[ -n "$IDF_PATH_ARG" ]]; then
    IDF_PATH="$IDF_PATH_ARG"
elif [[ -n "$IDF_PATH" ]]; then
    # Use existing IDF_PATH from environment
    :
elif [[ -d "$HOME/esp/esp-idf" ]]; then
    IDF_PATH="$HOME/esp/esp-idf"
else
    echo -e "${RED}Error: ESP-IDF not found. Specify with --idfpath or set IDF_PATH.${NC}"
    exit 1
fi

# Source ESP-IDF environment
echo -e "${YELLOW}Sourcing ESP-IDF from: $IDF_PATH${NC}"
source "$IDF_PATH/export.sh" > /dev/null 2>&1

# Skip build setup steps for flash-only mode
if [[ "$FLASH_ONLY" != true ]]; then
    # Setup BSEC library if path provided
    if [[ -n "$BSEC_PATH" ]]; then
        if [[ ! -f "$BSEC_PATH" ]]; then
            echo -e "${RED}Error: BSEC library not found: $BSEC_PATH${NC}"
            exit 1
        fi
        SETUP_SCRIPT="./components/bsec/setup_bsec.sh"
        if [[ ! -f "$SETUP_SCRIPT" ]]; then
            echo -e "${RED}Error: BSEC setup script not found: $SETUP_SCRIPT${NC}"
            exit 1
        fi
        echo -e "${YELLOW}Setting up BSEC library from: $BSEC_PATH${NC}"
        "$SETUP_SCRIPT" "$BSEC_PATH"
    fi

    # Clean if requested
    if [[ "$CLEAN" == true ]]; then
        echo -e "${YELLOW}Cleaning build directory: $BUILD_DIR${NC}"
        rm -rf "$BUILD_DIR"
    fi

    # Set target if specified or if build directory doesn't exist
    if [[ -n "$TARGET" ]]; then
        echo -e "${YELLOW}Setting target: $TARGET${NC}"
        idf.py -B "$BUILD_DIR" set-target "$TARGET"
    elif [[ ! -d "$BUILD_DIR" ]]; then
        echo -e "${RED}Error: Build directory '$BUILD_DIR' doesn't exist. Specify --target to create it.${NC}"
        exit 1
    fi
fi

# Output binary path
OUTPUT_BINARY="$BUILD_DIR/esp32-can-sensor.bin"

# Flash-only mode: skip build, just flash existing binary
if [[ "$FLASH_ONLY" == true ]]; then
    if [[ ! -d "$BUILD_DIR" ]]; then
        echo -e "${RED}Error: Build directory not found: $BUILD_DIR${NC}"
        echo -e "${RED}Build first with: $0 --target=<target>${NC}"
        exit 1
    fi
    if [[ ! -f "$OUTPUT_BINARY" ]]; then
        echo -e "${RED}Error: Binary not found: $OUTPUT_BINARY${NC}"
        echo -e "${RED}Build first with: $0 --target=<target>${NC}"
        exit 1
    fi
    SIZE=$(stat -f%z "$OUTPUT_BINARY" 2>/dev/null || stat -c%s "$OUTPUT_BINARY" 2>/dev/null)
    SIZE_KB=$((SIZE / 1024))
    echo -e "${GREEN}Using existing binary: $OUTPUT_BINARY (${SIZE_KB} KB)${NC}"
else
    # Build version arguments
    VERSION_ARGS=""
    if [[ -n "$VERSION_MAJOR" ]]; then
        VERSION_ARGS="-DFIRMWARE_VERSION_MAJOR=$VERSION_MAJOR"
    fi
    if [[ -n "$VERSION_MINOR" ]]; then
        VERSION_ARGS="$VERSION_ARGS -DFIRMWARE_VERSION_MINOR=$VERSION_MINOR"
    fi
    if [[ -n "$VERSION_PATCH" ]]; then
        VERSION_ARGS="$VERSION_ARGS -DFIRMWARE_VERSION_PATCH=$VERSION_PATCH"
    fi

    # Build
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  Building ESP32-CAN-Sensor firmware${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""

    idf.py -B "$BUILD_DIR" build $VERSION_ARGS

    if [[ -f "$OUTPUT_BINARY" ]]; then
        SIZE=$(stat -f%z "$OUTPUT_BINARY" 2>/dev/null || stat -c%s "$OUTPUT_BINARY" 2>/dev/null)
        SIZE_KB=$((SIZE / 1024))
        echo ""
        echo -e "${GREEN}Build successful!${NC}"
        echo -e "Binary: $OUTPUT_BINARY"
        echo -e "Size: ${SIZE_KB} KB ($SIZE bytes)"
    else
        echo -e "${RED}Error: Build binary not found: $OUTPUT_BINARY${NC}"
        exit 1
    fi
fi

# Flash if requested
if [[ "$FLASH" == true ]]; then
    FLASH_ARGS="-B $BUILD_DIR"
    if [[ -n "$PORT" ]]; then
        FLASH_ARGS="$FLASH_ARGS -p $PORT"
    fi
    echo ""
    echo -e "${YELLOW}Flashing firmware...${NC}"
    idf.py $FLASH_ARGS flash
fi

# Monitor if requested
if [[ "$MONITOR" == true ]]; then
    MONITOR_ARGS="-B $BUILD_DIR"
    if [[ -n "$PORT" ]]; then
        MONITOR_ARGS="$MONITOR_ARGS -p $PORT"
    fi
    echo ""
    echo -e "${YELLOW}Starting monitor...${NC}"
    idf.py $MONITOR_ARGS monitor
fi
