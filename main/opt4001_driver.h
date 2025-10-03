/* opt4001_driver.h - OPT4001 Ambient Light Sensor Driver */

#ifndef OPT4001_DRIVER_H
#define OPT4001_DRIVER_H

#include <esp_err.h>
#include <stdint.h>

/**
 * @brief Initialize OPT4001 sensor
 *
 * Configures I2C communication and sets up the sensor with default settings:
 * - Auto-ranging enabled (hardware manages gain automatically)
 * - Conversion time: 800ms (highest accuracy)
 * - Continuous conversion mode
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t opt4001_init(void);

/**
 * @brief Read lux value from OPT4001
 *
 * The OPT4001 handles auto-ranging automatically in hardware, so this
 * function simply reads the result register and converts to lux.
 * No manual configuration switching or calibration needed.
 *
 * Range: 0.001 to 2,200,000 lux
 *
 * @param[out] lux Pointer to store lux value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t opt4001_read_lux(float *lux);

/**
 * @brief Get current auto-range index
 *
 * The OPT4001 has 12 auto-range settings (0-11) that it switches
 * between automatically. This function returns the current range.
 *
 * @return Current range index (0-11), or 255 on error
 */
uint8_t opt4001_get_range_index(void);

/**
 * @brief Get sensor status string
 *
 * @return Status string ("OK", "Not Initialized", etc.)
 */
const char* opt4001_get_status_string(void);

#endif // OPT4001_DRIVER_H
