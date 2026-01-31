/* opt3001_driver.h - OPT3001 Ambient Light Sensor Driver */

#ifndef OPT3001_DRIVER_H
#define OPT3001_DRIVER_H

#include <esp_err.h>
#include <stdint.h>

/**
 * @brief Initialize OPT3001 sensor
 *
 * Configures I2C communication and sets up the sensor with default settings:
 * - Auto-ranging enabled (hardware manages gain automatically)
 * - Conversion time: 800ms (highest accuracy)
 * - Continuous conversion mode
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t opt3001_init(void);

/**
 * @brief Read lux value from OPT3001
 *
 * The OPT3001 handles auto-ranging automatically in hardware, so this
 * function simply reads the result register and converts to lux.
 * No manual configuration switching or calibration needed.
 *
 * Formula: lux = 0.01 × 2^E × R
 * Where E = exponent (bits 15:12), R = mantissa (bits 11:0)
 *
 * Range: 0.01 to 83,865.6 lux
 *
 * @param[out] lux Pointer to store lux value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t opt3001_read_lux(float *lux);

/**
 * @brief Get current auto-range index (exponent)
 *
 * The OPT3001 has 12 auto-range settings (0-11) that it switches
 * between automatically. This function returns the current range.
 *
 * @return Current range index (0-11), or 255 on error
 */
uint8_t opt3001_get_range_index(void);

/**
 * @brief Get sensor status string
 *
 * @return Status string ("OK", "Not Initialized", etc.)
 */
const char* opt3001_get_status_string(void);

#endif // OPT3001_DRIVER_H
