/* als_driver.c - Unified Ambient Light Sensor Driver Implementation */

#include "als_driver.h"
#include "veml7700_driver.h"
#include "opt4001_driver.h"
#include <driver/i2c.h>
#include <esp_log.h>

static const char *TAG = "ALS";

/* ======================== Hardware Configuration ======================== */

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_SDA_GPIO_NUM        GPIO_NUM_6
#define I2C_SCL_GPIO_NUM        GPIO_NUM_7

/* ======================== Static State ======================== */

static als_type_t detected_sensor = ALS_TYPE_NONE;

/* ======================== I2C Initialization ======================== */

/**
 * @brief Initialize I2C bus for sensor communication
 *
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO_NUM,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    /* ESP_ERR_INVALID_STATE means driver already installed - that's OK */
    return ESP_OK;
}

/* ======================== I2C Probing ======================== */

/**
 * @brief Probe I2C bus for device at given address
 *
 * @param addr 7-bit I2C address
 * @return ESP_OK if device responds, error otherwise
 */
static esp_err_t i2c_probe_device(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/* ======================== Public API ======================== */

esp_err_t als_init(void) {
    ESP_LOGI(TAG, "Initializing I2C bus...");

    /* Initialize I2C before probing */
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return ret;
    }

    ESP_LOGI(TAG, "Auto-detecting ambient light sensor...");

    /* Try VEML7700 first (address 0x10) */
    if (i2c_probe_device(0x10) == ESP_OK) {
        ESP_LOGI(TAG, "VEML7700 detected at address 0x10");
        esp_err_t ret = veml7700_init();
        if (ret == ESP_OK) {
            detected_sensor = ALS_TYPE_VEML7700;
            ESP_LOGI(TAG, "Using VEML7700 (0-120K lux, with calibration)");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "VEML7700 initialization failed");
        }
    }

    /* Try OPT4001 (address 0x44) */
    if (i2c_probe_device(0x44) == ESP_OK) {
        ESP_LOGI(TAG, "OPT4001 detected at address 0x44");
        esp_err_t ret = opt4001_init();
        if (ret == ESP_OK) {
            detected_sensor = ALS_TYPE_OPT4001;
            ESP_LOGI(TAG, "Using OPT4001 (0-2.2M lux, factory calibrated)");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "OPT4001 initialization failed");
        }
    }

    /* No sensor found */
    ESP_LOGE(TAG, "No ambient light sensor detected!");
    ESP_LOGE(TAG, "Checked: 0x10 (VEML7700), 0x44 (OPT4001)");
    detected_sensor = ALS_TYPE_NONE;
    return ESP_ERR_NOT_FOUND;
}

esp_err_t als_read_lux(float *lux) {
    if (detected_sensor == ALS_TYPE_NONE) {
        ESP_LOGE(TAG, "No sensor initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (lux == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (detected_sensor) {
        case ALS_TYPE_VEML7700:
            /* VEML7700: Driver includes calibration correction */
            return veml7700_read_lux(lux);

        case ALS_TYPE_OPT4001:
            /* OPT4001: Factory calibrated, no correction needed */
            return opt4001_read_lux(lux);

        default:
            return ESP_ERR_INVALID_STATE;
    }
}

als_type_t als_get_sensor_type(void) {
    return detected_sensor;
}

uint8_t als_get_config_idx(void) {
    switch (detected_sensor) {
        case ALS_TYPE_VEML7700:
            /* VEML7700: Config index 0-20 */
            return veml7700_get_config_idx();

        case ALS_TYPE_OPT4001:
            /* OPT4001: Auto-range index 0-11, offset by 100 to distinguish from VEML7700
             * This allows CAN clients to identify sensor type:
             * - 0-20: VEML7700
             * - 100-111: OPT4001
             */
            return 100 + opt4001_get_range_index();

        default:
            return 0;
    }
}

const char* als_get_status_string(void) {
    switch (detected_sensor) {
        case ALS_TYPE_VEML7700:
            return veml7700_get_status_string();

        case ALS_TYPE_OPT4001:
            return opt4001_get_status_string();

        default:
            return "No Sensor";
    }
}

const char* als_get_sensor_name(void) {
    switch (detected_sensor) {
        case ALS_TYPE_VEML7700:
            return "VEML7700";

        case ALS_TYPE_OPT4001:
            return "OPT4001";

        default:
            return "None";
    }
}
