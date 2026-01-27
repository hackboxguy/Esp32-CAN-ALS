# main/CMakeLists.txt
#
# Supports two firmware types:
#   FACTORY - Minimal firmware with OTA support only
#   MAIN    - Full-featured firmware with all sensors
#
# Build with: idf.py build -DFIRMWARE_TYPE=FACTORY
#         or: idf.py build -DFIRMWARE_TYPE=MAIN (default)

# Check firmware type from command line (default to MAIN)
if(NOT DEFINED FIRMWARE_TYPE)
    set(FIRMWARE_TYPE "MAIN")
endif()

message(STATUS "")
message(STATUS "========================================")
message(STATUS "  Firmware Type: ${FIRMWARE_TYPE}")
message(STATUS "========================================")
message(STATUS "")

# Base sources (always included for both FACTORY and MAIN)
set(SRCS
    "main.c"
    "can_protocol.c"
    "ota_handler.c"
)

# Base dependencies (always required)
set(REQUIRES
    driver
    esp_timer
    nvs_flash
    app_update
    esp_partition
)

# Sensor sources and dependencies (MAIN only)
if(FIRMWARE_TYPE STREQUAL "MAIN")
    message(STATUS "Including sensor drivers (MAIN build)")
    list(APPEND SRCS
        "als_driver.c"
        "veml7700_driver.c"
        "opt4001_driver.c"
        "opt3001_driver.c"
        "bme680_bsec.c"
    )
    # BSEC library required for BME680
    list(APPEND REQUIRES bsec)
else()
    message(STATUS "Excluding sensor drivers (FACTORY build)")
endif()

# Register the component
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)

# Add firmware type as compile definition (after component registration)
if(FIRMWARE_TYPE STREQUAL "FACTORY")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIRMWARE_TYPE=0)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIRMWARE_TYPE=1)
endif()

# Add firmware version as compile definitions if provided via -D flags
if(DEFINED FIRMWARE_VERSION_MAJOR)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIRMWARE_VERSION_MAJOR=${FIRMWARE_VERSION_MAJOR})
    message(STATUS "Firmware version major: ${FIRMWARE_VERSION_MAJOR}")
endif()
if(DEFINED FIRMWARE_VERSION_MINOR)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIRMWARE_VERSION_MINOR=${FIRMWARE_VERSION_MINOR})
    message(STATUS "Firmware version minor: ${FIRMWARE_VERSION_MINOR}")
endif()
if(DEFINED FIRMWARE_VERSION_PATCH)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIRMWARE_VERSION_PATCH=${FIRMWARE_VERSION_PATCH})
    message(STATUS "Firmware version patch: ${FIRMWARE_VERSION_PATCH}")
endif()
