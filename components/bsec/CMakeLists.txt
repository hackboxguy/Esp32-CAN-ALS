# BSEC Library Component for ESP-IDF
#
# This component is OPTIONAL and requires user setup.
# See README.md for installation instructions.

set(COMPONENT_SRCS "")
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_PRIV_INCLUDEDIRS "")
set(COMPONENT_REQUIRES "")

# Check if BSEC library files are present
set(BSEC_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/libalgobsec.a")
set(BSEC_HEADER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/bsec_interface.h")
set(BME68X_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/bme68x.c")

if(EXISTS "${BSEC_LIB_PATH}" AND
   EXISTS "${BSEC_HEADER_PATH}" AND
   EXISTS "${BME68X_SRC_PATH}")

    message(STATUS "BSEC library found - BME680 support available")

    # Add source files
    set(COMPONENT_SRCS
        "src/bme68x.c"
    )

    # Add include directories
    set(COMPONENT_ADD_INCLUDEDIRS
        "include"
    )

    # Register component
    idf_component_register(
        SRCS ${COMPONENT_SRCS}
        INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
        REQUIRES driver esp_timer nvs_flash
    )

    # Link precompiled BSEC library
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-L${CMAKE_CURRENT_SOURCE_DIR}/lib")
    target_link_libraries(${COMPONENT_LIB} PUBLIC "-lalgobsec")

    # Add compile definitions to indicate BSEC is available
    target_compile_definitions(${COMPONENT_LIB} PUBLIC BSEC_LIBRARY_AVAILABLE=1)

    message(STATUS "BSEC library linked: ${BSEC_LIB_PATH}")

else()

    message(WARNING "========================================")
    message(WARNING "  BSEC library NOT found")
    message(WARNING "  BME680 support DISABLED")
    message(WARNING "========================================")
    message(WARNING "To enable BME680/BME688 support:")
    message(WARNING "  1. Download BSEC from Bosch Sensortec")
    message(WARNING "  2. Run: components/bsec/setup_bsec.sh <path_to_zip>")
    message(WARNING "  3. Run: idf.py menuconfig")
    message(WARNING "  4. Enable: Component config → BSEC → Enable BME680")
    message(WARNING "")
    message(WARNING "See components/bsec/README.md for details")
    message(WARNING "========================================")

    # Register empty component (no sources)
    idf_component_register(
        SRCS ""
        INCLUDE_DIRS ""
    )

    # Define that BSEC is NOT available
    target_compile_definitions(${COMPONENT_LIB} PUBLIC BSEC_LIBRARY_AVAILABLE=0)

endif()
