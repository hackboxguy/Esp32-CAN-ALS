/* veml7700_driver.h - VEML7700 Ambient Light Sensor Driver */

#ifndef VEML7700_DRIVER_H
#define VEML7700_DRIVER_H

#include <esp_err.h>
#include <stdint.h>

/**
 * @brief Initialize VEML7700 sensor and I2C interface
 *
 * Configures I2C bus (GPIO6/7) and initializes sensor with default settings.
 * Default config: 1x gain, 100ms integration time (config index 7).
 *
 * @return ESP_OK on success, ESP_FAIL on I2C or sensor initialization failure
 */
esp_err_t veml7700_init(void);

/**
 * @brief Read calibrated lux value with intelligent auto-ranging
 *
 * Reads sensor, applies auto-ranging algorithm, moving average filter,
 * and calibration correction for accurate lux measurement.
 *
 * Features:
 * - Intelligent auto-ranging (18 configurations, gain × integration time)
 * - Moving average filter (8 samples)
 * - Piecewise linear calibration correction
 * - Settling time after configuration changes
 *
 * @param[out] lux Pointer to store calibrated lux value (0-120,000 lux typical)
 * @return ESP_OK on success, ESP_FAIL on I2C read failure
 */
esp_err_t veml7700_read_lux(float *lux);

/**
 * @brief Get current configuration index
 *
 * Returns the current active configuration (0-17), useful for debugging
 * auto-ranging behavior.
 *
 * Configuration indices:
 * - 0-4: 2x gain (highest sensitivity, lowest lux range)
 * - 5-10: 1x gain (medium sensitivity)
 * - 11-14: 1/4x gain (lower sensitivity)
 * - 15-17: 1/8x gain (lowest sensitivity, highest lux range)
 *
 * @return Current configuration index (0-17)
 */
uint8_t veml7700_get_config_idx(void);

/**
 * @brief Get human-readable status string
 *
 * @return Status string (e.g., "OK", "I2C Error", "Initializing")
 */
const char* veml7700_get_status_string(void);

#endif // VEML7700_DRIVER_H
