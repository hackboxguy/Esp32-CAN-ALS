# main/CMakeLists.txt
#
# ESP32-CAN-Sensor firmware with full sensor support

# Source files
set(SRCS
    "main.c"
    "can_protocol.c"
    "ota_handler.c"
    "als_driver.c"
    "veml7700_driver.c"
    "opt4001_driver.c"
    "opt3001_driver.c"
    "bme680_bsec.c"
)

# Dependencies
set(REQUIRES
    driver
    esp_timer
    nvs_flash
    app_update
    esp_partition
    bsec
)

# Register the component
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)

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
