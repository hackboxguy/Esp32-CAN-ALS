/* opt3001_driver.c - OPT3001 Ambient Light Sensor Driver Implementation */

#include "opt3001_driver.h"
#include <driver/i2c.h>
#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "OPT3001";

/* ======================== Hardware Configuration ======================== */

#define I2C_MASTER_NUM          I2C_NUM_0

/* OPT3001 I2C Addresses (configurable via ADDR pin)
 * 0x44 = ADDR to GND
 * 0x45 = ADDR to VDD
 * 0x46 = ADDR to SDA
 * 0x47 = ADDR to SCL
 */
#define OPT3001_I2C_ADDR        0x44

/* OPT3001 Register Addresses */
#define OPT3001_REG_RESULT          0x00    // Result register (lux data)
#define OPT3001_REG_CONFIG          0x01    // Configuration register
#define OPT3001_REG_LOW_LIMIT       0x02    // Low limit register
#define OPT3001_REG_HIGH_LIMIT      0x03    // High limit register
#define OPT3001_REG_MANUFACTURER_ID 0x7E    // Manufacturer ID register
#define OPT3001_REG_DEVICE_ID       0x7F    // Device ID register

/* Configuration Register Bits (reset value = 0xC810) */
#define OPT3001_CFG_RANGE_AUTO      (0x0C << 12)  // Auto-range mode (RN[3:0] = 1100b)
#define OPT3001_CFG_CONV_800MS      (1 << 11)     // 800ms conversion time (CT = 1)
#define OPT3001_CFG_CONV_100MS      (0 << 11)     // 100ms conversion time (CT = 0)
#define OPT3001_CFG_MODE_SHUTDOWN   (0x00 << 9)   // Shutdown mode (M[1:0] = 00)
#define OPT3001_CFG_MODE_SINGLE     (0x01 << 9)   // Single-shot mode (M[1:0] = 01)
#define OPT3001_CFG_MODE_CONT       (0x02 << 9)   // Continuous conversion (M[1:0] = 10 or 11)
#define OPT3001_CFG_LATCH           (1 << 4)      // Latch mode (L = 1)
#define OPT3001_CFG_POL_LOW         (0 << 3)      // INT polarity active low
#define OPT3001_CFG_FAULT_CNT_1     (0x00)        // Fault count = 1

/* Expected Device IDs */
#define OPT3001_MANUFACTURER_ID     0x5449  // "TI" in ASCII
#define OPT3001_DEVICE_ID           0x3001  // OPT3001 device ID

/* Result register bit masks */
#define OPT3001_EXPONENT_MASK       0xF000  // Bits 15:12
#define OPT3001_EXPONENT_SHIFT      12
#define OPT3001_MANTISSA_MASK       0x0FFF  // Bits 11:0

/* ======================== Static State ======================== */

static bool initialized = false;
static uint8_t last_range_index = 0;

/* ======================== I2C Communication ======================== */

static esp_err_t opt3001_write_reg(uint8_t reg, uint16_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OPT3001_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true);  // MSB first (BIG ENDIAN)
    i2c_master_write_byte(cmd, data & 0xFF, true);         // LSB second
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write error reg=0x%02X, data=0x%04X: %s", reg, data, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "I2C write OK reg=0x%02X, data=0x%04X", reg, data);
    }

    return ret;
}

static esp_err_t opt3001_read_reg(uint8_t reg, uint16_t *data) {
    uint8_t data_rd[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OPT3001_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OPT3001_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data_rd[0], I2C_MASTER_ACK);  // MSB first (BIG ENDIAN)
    i2c_master_read_byte(cmd, &data_rd[1], I2C_MASTER_NACK); // LSB second
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *data = (data_rd[0] << 8) | data_rd[1];  // MSB << 8 | LSB
    } else {
        ESP_LOGE(TAG, "I2C read error reg=0x%02X: %s", reg, esp_err_to_name(ret));
    }

    return ret;
}

/* ======================== Public API ======================== */

esp_err_t opt3001_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* Verify Manufacturer ID */
    uint16_t mfr_id;
    esp_err_t ret = opt3001_read_reg(OPT3001_REG_MANUFACTURER_ID, &mfr_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read manufacturer ID");
        return ret;
    }

    if (mfr_id != OPT3001_MANUFACTURER_ID) {
        ESP_LOGE(TAG, "Invalid manufacturer ID: 0x%04X (expected 0x%04X)", mfr_id, OPT3001_MANUFACTURER_ID);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Manufacturer ID verified: 0x%04X (TI)", mfr_id);

    /* Verify Device ID */
    uint16_t device_id;
    ret = opt3001_read_reg(OPT3001_REG_DEVICE_ID, &device_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }

    if (device_id != OPT3001_DEVICE_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%04X (expected 0x%04X)", device_id, OPT3001_DEVICE_ID);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Device ID verified: 0x%04X", device_id);

    /* Configure sensor:
     * - Auto-range mode (RN[3:0] = 1100b, hardware manages gain automatically)
     * - 800ms conversion time (CT = 1, highest accuracy)
     * - Continuous conversion mode (M[1:0] = 10b)
     * - Latch mode enabled
     * - Fault count = 1
     */
    uint16_t config = OPT3001_CFG_RANGE_AUTO |
                      OPT3001_CFG_CONV_800MS |
                      OPT3001_CFG_MODE_CONT |
                      OPT3001_CFG_LATCH |
                      OPT3001_CFG_POL_LOW |
                      OPT3001_CFG_FAULT_CNT_1;

    ret = opt3001_write_reg(OPT3001_REG_CONFIG, config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "Config written: 0x%04X", config);

    /* Wait for first conversion (800ms + margin) */
    vTaskDelay(pdMS_TO_TICKS(900));

    initialized = true;
    ESP_LOGI(TAG, "Initialized successfully (auto-range, 800ms, continuous)");

    return ESP_OK;
}

esp_err_t opt3001_read_lux(float *lux) {
    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (lux == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read result register (16-bit) */
    uint16_t result;
    esp_err_t ret = opt3001_read_reg(OPT3001_REG_RESULT, &result);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Extract exponent (bits 15:12) and mantissa (bits 11:0) */
    uint8_t exponent = (result >> OPT3001_EXPONENT_SHIFT) & 0x0F;
    uint16_t mantissa = result & OPT3001_MANTISSA_MASK;

    /* Store range index for status reporting */
    last_range_index = exponent;

    /* Calculate lux value
     * Formula: lux = 0.01 × 2^exponent × mantissa
     *
     * The OPT3001 has 12 ranges (exponent 0-11):
     * - Range 0 (exp=0): LSB = 0.01 lux, full-scale = 40.95 lux
     * - Range 11 (exp=11): LSB = 20.48 lux, full-scale = 83,865.6 lux
     */
    float lsb_size = 0.01f * powf(2.0f, (float)exponent);
    *lux = lsb_size * (float)mantissa;

    ESP_LOGD(TAG, "Result=0x%04X, exp=%u, mantissa=%u, LSB=%.4f, lux=%.2f",
             result, exponent, mantissa, lsb_size, *lux);

    return ESP_OK;
}

uint8_t opt3001_get_range_index(void) {
    if (!initialized) {
        return 255;
    }
    return last_range_index;
}

const char* opt3001_get_status_string(void) {
    if (!initialized) {
        return "Not Initialized";
    }
    return "OK";
}
