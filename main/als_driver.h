/* als_driver.h - Unified Ambient Light Sensor Driver (VEML7700, OPT4001, or OPT3001) */

#ifndef ALS_DRIVER_H
#define ALS_DRIVER_H

#include <esp_err.h>
#include <stdint.h>

/**
 * @brief Ambient Light Sensor Types
 */
typedef enum {
    ALS_TYPE_NONE = 0,
    ALS_TYPE_VEML7700,
    ALS_TYPE_OPT4001,
    ALS_TYPE_OPT3001
} als_type_t;

/**
 * @brief Initialize ambient light sensor
 *
 * Auto-detects which sensor is present on the I2C bus:
 * - Probes 0x10 for VEML7700
 * - Probes 0x44 for OPT4001 or OPT3001 (distinguished by Device ID)
 * - Initializes whichever sensor is found
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no sensor detected
 */
esp_err_t als_init(void);

/**
 * @brief Read lux value from detected sensor
 *
 * Routes to appropriate sensor driver:
 * - VEML7700: Applies calibration correction for diffuser
 * - OPT4001: Returns raw value (factory calibrated)
 *
 * @param[out] lux Pointer to store lux value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t als_read_lux(float *lux);

/**
 * @brief Get detected sensor type
 *
 * @return ALS_TYPE_VEML7700, ALS_TYPE_OPT4001, ALS_TYPE_OPT3001, or ALS_TYPE_NONE
 */
als_type_t als_get_sensor_type(void);

/**
 * @brief Get configuration/range index for CAN reporting
 *
 * Returns current operating configuration:
 * - VEML7700: 0-20 (configuration index)
 * - OPT4001: 100-111 (auto-range index + 100 offset)
 * - OPT3001: 200-211 (auto-range index + 200 offset)
 *
 * This allows CAN clients to distinguish sensor type from config field.
 *
 * @return Configuration/range index
 */
uint8_t als_get_config_idx(void);

/**
 * @brief Get sensor status string
 *
 * @return Human-readable status string
 */
const char* als_get_status_string(void);

/**
 * @brief Get sensor type name
 *
 * @return "VEML7700", "OPT4001", "OPT3001", or "None"
 */
const char* als_get_sensor_name(void);

#endif // ALS_DRIVER_H
