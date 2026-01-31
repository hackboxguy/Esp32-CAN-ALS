/* bme680_bsec.h - BME680/BME688 Environmental Sensor with BSEC Integration
 *
 * Unified driver supporting both BME680 and BME688 sensors.
 * Uses Bosch BSEC library for accurate air quality measurements.
 *
 * This driver automatically detects BME680 vs BME688 at runtime.
 * All APIs work identically for both sensor variants.
 */

#ifndef BME680_BSEC_H
#define BME680_BSEC_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BME68x sensor data structure */
typedef struct {
    /* Environmental measurements */
    float temperature;      /* Temperature in °C */
    float humidity;         /* Relative humidity in %RH */
    float pressure;         /* Barometric pressure in hPa */

    /* Air quality measurements (from BSEC) */
    uint16_t iaq;          /* Indoor Air Quality index (0-500) */
    uint8_t accuracy;      /* IAQ accuracy (0-3, 3=fully calibrated) */
    uint16_t co2_equiv;    /* CO₂ equivalent in ppm */
    uint16_t breath_voc;   /* Breath VOC equivalent in ppm */

    /* Sensor status */
    bool is_bme688;        /* true=BME688, false=BME680 */
    uint32_t timestamp_ms; /* Measurement timestamp */
} bme68x_data_t;

/* BSEC calibration accuracy levels */
#define BSEC_ACCURACY_UNRELIABLE    0  /* Sensor stabilizing */
#define BSEC_ACCURACY_LOW           1  /* Low accuracy, calibrating */
#define BSEC_ACCURACY_MEDIUM        2  /* Medium accuracy, partially calibrated */
#define BSEC_ACCURACY_HIGH          3  /* High accuracy, fully calibrated */

/* IAQ index ranges (Bosch classification) */
#define IAQ_EXCELLENT_MAX       50   /* 0-50: Excellent air quality */
#define IAQ_GOOD_MAX           100   /* 51-100: Good */
#define IAQ_LIGHTLY_POLLUTED   150   /* 101-150: Lightly polluted */
#define IAQ_MODERATELY_POLLUTED 200  /* 151-200: Moderately polluted */
#define IAQ_HEAVILY_POLLUTED    250  /* 201-250: Heavily polluted */
#define IAQ_SEVERELY_POLLUTED   350  /* 251-350: Severely polluted */
                                     /* 351-500: Extremely polluted */

/**
 * @brief Initialize BME680/BME688 sensor and BSEC library
 *
 * Performs the following:
 * - Initializes I2C communication with sensor
 * - Auto-detects BME680 vs BME688 variant
 * - Initializes BSEC library
 * - Loads calibration state from NVS (if available)
 * - Configures sensor for continuous measurement
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_NOT_FOUND: Sensor not detected on I2C bus
 *     - ESP_ERR_INVALID_RESPONSE: Sensor communication error
 *     - ESP_FAIL: BSEC initialization failed
 */
esp_err_t bme680_init(void);

/**
 * @brief Read environmental and air quality data from sensor
 *
 * This function should be called periodically (every 3 seconds recommended).
 * BSEC requires regular sampling for accurate IAQ calculation.
 *
 * Note: First 5 minutes after power-on, IAQ accuracy will be low (0-1)
 *       as BSEC learns baseline. Full accuracy takes up to 28 days.
 *
 * @param data Pointer to structure to store sensor readings
 *
 * @return
 *     - ESP_OK: Success, data contains valid readings
 *     - ESP_ERR_INVALID_ARG: data pointer is NULL
 *     - ESP_ERR_INVALID_STATE: Sensor not initialized
 *     - ESP_FAIL: Sensor read error
 */
esp_err_t bme680_read(bme68x_data_t *data);

/**
 * @brief Get sensor variant name
 *
 * @return String "BME680" or "BME688" (auto-detected at init)
 */
const char* bme680_get_variant_name(void);

/**
 * @brief Get BSEC library version
 *
 * @return BSEC version string (e.g. "2.6.1.0")
 */
const char* bme680_get_bsec_version(void);

/**
 * @brief Save current BSEC calibration state to NVS
 *
 * BSEC learns air quality baselines over time. Saving state preserves
 * calibration across reboots.
 *
 * Should be called periodically (every 4 hours recommended) to balance
 * flash wear leveling with state freshness.
 *
 * @return
 *     - ESP_OK: State saved successfully
 *     - ESP_ERR_INVALID_STATE: BSEC not initialized
 *     - ESP_ERR_NVS_*: NVS write error
 */
esp_err_t bme680_save_state(void);

/**
 * @brief Reset BSEC calibration state (force recalibration)
 *
 * Clears stored calibration from NVS and resets BSEC to initial state.
 * Sensor will recalibrate from scratch (takes up to 28 days for full accuracy).
 *
 * Use this if:
 * - Moving sensor to different environment
 * - IAQ readings seem incorrect
 * - Troubleshooting calibration issues
 *
 * @return
 *     - ESP_OK: State reset successfully
 *     - ESP_ERR_NVS_*: NVS erase error
 */
esp_err_t bme680_reset_calibration(void);

/**
 * @brief Get I2C address of detected sensor
 *
 * @return I2C address (0x76 or 0x77), or 0 if not initialized
 */
uint8_t bme680_get_i2c_address(void);

/**
 * @brief Check if sensor is BME688 variant
 *
 * @return true if BME688, false if BME680
 */
bool bme680_is_bme688(void);

/**
 * @brief Get time since last BSEC state save (in seconds)
 *
 * @return Seconds since last save, or 0 if never saved
 */
uint32_t bme680_get_state_age(void);

/**
 * @brief Get next BSEC call time in milliseconds
 *
 * Call this to determine when to next call bme680_read().
 * BSEC operates on precise timing intervals.
 *
 * @return Next call time in milliseconds from boot, or 0 if not initialized
 */
uint64_t bme680_get_next_call_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* BME680_BSEC_H */
