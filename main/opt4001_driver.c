/* opt4001_driver.c - OPT4001 Ambient Light Sensor Driver Implementation */

#include "opt4001_driver.h"
#include <driver/i2c.h>
#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "OPT4001";

/* ======================== Hardware Configuration ======================== */

#define I2C_MASTER_NUM          I2C_NUM_0
#define OPT4001_I2C_ADDR        0x44

/* OPT4001 Register Addresses */
#define OPT4001_REG_RESULT      0x00    // Result register (lux data) - 32-bit
#define OPT4001_REG_CONFIG      0x0A    // Configuration register
#define OPT4001_REG_FLAGS       0x0B    // Flags register
#define OPT4001_REG_DEVICE_ID   0x11    // Device ID register

/* Configuration Register Bits */
#define OPT4001_CFG_QWAKE       (1 << 15)   // Quick wake
#define OPT4001_CFG_RANGE_AUTO  (0x0C << 10) // Auto-range mode
#define OPT4001_CFG_CONV_800MS  (0x08 << 6)  // 800ms conversion time
#define OPT4001_CFG_MODE_CONT   (0x03 << 4)  // Continuous conversion
#define OPT4001_CFG_LATCH       (1 << 3)     // Latch mode
#define OPT4001_CFG_INT_POL     (0 << 2)     // Interrupt polarity (active low)
#define OPT4001_CFG_FAULT_CNT   (0x01)       // Fault count (1 fault)

/* Expected Device ID (bits 11:0 of register 0x11) */
#define OPT4001_DEVICE_ID_MASK  0x0FFF
#define OPT4001_DEVICE_ID       0x0121

/* ======================== Static State ======================== */

static bool initialized = false;
static uint8_t last_range_index = 0;

/* ======================== I2C Communication ======================== */

static esp_err_t opt4001_write_reg(uint8_t reg, uint16_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OPT4001_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
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

static esp_err_t opt4001_read_reg(uint8_t reg, uint16_t *data) {
    uint8_t data_rd[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OPT4001_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OPT4001_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data_rd[0], I2C_MASTER_ACK);  // MSB first (BIG ENDIAN)
    i2c_master_read_byte(cmd, &data_rd[1], I2C_MASTER_NACK); // LSB second
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *data = (data_rd[0] << 8) | data_rd[1];  // MSB << 8 | LSB
    } else {
        ESP_LOGE(TAG, "I2C read error: %s", esp_err_to_name(ret));
    }

    return ret;
}

/* OPT4001 Register Addresses for result (matches Linux driver) */
#define OPT4001_LIGHT1_MSB    0x00
#define OPT4001_LIGHT1_LSB    0x01

/* OPT4001 result register bit masks */
#define OPT4001_EXPONENT_MASK    0xF000  // Bits 15:12
#define OPT4001_MSB_RESULT_MASK  0x0FFF  // Bits 11:0
#define OPT4001_LSB_RESULT_MASK  0xFF00  // Bits 15:8

/* ======================== Public API ======================== */

esp_err_t opt4001_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* Verify device ID */
    uint16_t device_id;
    esp_err_t ret = opt4001_read_reg(OPT4001_REG_DEVICE_ID, &device_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }

    /* Mask off upper bits (only bits 11:0 contain device ID) */
    device_id &= OPT4001_DEVICE_ID_MASK;

    if (device_id != OPT4001_DEVICE_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%03X (expected 0x%03X)", device_id, OPT4001_DEVICE_ID);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Device ID verified: 0x%03X", device_id);

    /* Configure sensor:
     * - Auto-range mode (hardware manages gain automatically)
     * - 800ms conversion time (highest accuracy)
     * - Continuous conversion mode
     * - Latch interrupts
     * - Fault count = 1
     */
    uint16_t config = OPT4001_CFG_RANGE_AUTO |
                      OPT4001_CFG_CONV_800MS |
                      OPT4001_CFG_MODE_CONT |
                      OPT4001_CFG_LATCH |
                      OPT4001_CFG_INT_POL |
                      OPT4001_CFG_FAULT_CNT;

    ret = opt4001_write_reg(OPT4001_REG_CONFIG, config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Wait for first conversion (800ms + margin) */
    vTaskDelay(pdMS_TO_TICKS(900));

    initialized = true;
    ESP_LOGI(TAG, "Initialized successfully (auto-range, 800ms, continuous)");

    return ESP_OK;
}

esp_err_t opt4001_read_lux(float *lux) {
    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (lux == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read MSB and LSB registers (matches Linux driver approach) */
    uint16_t light_msb, light_lsb;
    esp_err_t ret = opt4001_read_reg(OPT4001_LIGHT1_MSB, &light_msb);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = opt4001_read_reg(OPT4001_LIGHT1_LSB, &light_lsb);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Extract fields from the two registers */
    uint8_t exponent = (light_msb >> 12) & 0x0F;        // Bits 15:12 of MSB
    uint16_t msb_data = light_msb & 0x0FFF;             // Bits 11:0 of MSB
    uint8_t lsb_data = (light_lsb >> 8) & 0xFF;         // Bits 15:8 of LSB

    /* Combine into 20-bit mantissa */
    uint32_t mantissa = (msb_data << 8) | lsb_data;

    /* Store range index for status reporting */
    last_range_index = exponent;

    /* Calculate lux value
     * Formula: lux = mantissa * 2^exponent * (4375 / 10000000)
     * The constant depends on OPT4001 packaging:
     * - SOT-5x3: 4375 / 10000000 = 0.0004375
     * - PICOSTAR: 3125 / 10000000 = 0.0003125
     *
     * Using SOT-5x3 constant (most common package)
     * The OPT4001 has 12 ranges (exponent 0-11)
     * Range 0: 0.001 - 5.24 lux
     * Range 11: 2621 - 2,200,000 lux
     */
    *lux = (float)mantissa * powf(2.0f, (float)exponent) * 0.0004375f;

    ESP_LOGD(TAG, "MSB=0x%04X, LSB=0x%04X, exp=%u, mantissa=%lu, lux=%.1f",
             light_msb, light_lsb, exponent, mantissa, *lux);

    return ESP_OK;
}

uint8_t opt4001_get_range_index(void) {
    if (!initialized) {
        return 255;
    }
    return last_range_index;
}

const char* opt4001_get_status_string(void) {
    if (!initialized) {
        return "Not Initialized";
    }
    return "OK";
}
