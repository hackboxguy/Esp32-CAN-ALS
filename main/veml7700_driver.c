/* veml7700_driver.c - VEML7700 Ambient Light Sensor Driver Implementation */

#include "veml7700_driver.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>

static const char *TAG = "VEML7700";

/* ======================== Hardware Configuration ======================== */

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_SDA_GPIO_NUM        GPIO_NUM_6
#define I2C_SCL_GPIO_NUM        GPIO_NUM_7

#define VEML7700_I2C_ADDR       0x10
#define VEML7700_REG_CONF       0x00
#define VEML7700_REG_ALS        0x04

/* VEML7700 Configuration Bits */
#define VEML7700_GAIN_1X        (0x00 << 11)
#define VEML7700_GAIN_2X        (0x01 << 11)
#define VEML7700_GAIN_1_4X      (0x02 << 11)
#define VEML7700_GAIN_1_8X      (0x03 << 11)

#define VEML7700_IT_25MS        0x0300
#define VEML7700_IT_50MS        0x0200
#define VEML7700_IT_100MS       0x0000
#define VEML7700_IT_200MS       0x0040
#define VEML7700_IT_400MS       0x0080
#define VEML7700_IT_800MS       0x00C0

#define VEML7700_POWER_ON       0x0000
#define VEML7700_POWER_OFF      0x0001

/* Auto-ranging parameters */
#define SATURATED_HIGH          60000
#define OPTIMAL_HIGH            40000
#define OPTIMAL_LOW             10000
#define TOO_LOW                 2000

#define READING_HISTORY_SIZE    5
#define SETTLE_READINGS_REQUIRED 3
#define CONSISTENCY_READINGS    3

#define MOVING_AVG_SAMPLES      8

/* ======================== Data Structures ======================== */

typedef struct {
    uint16_t gain_bits;
    uint16_t it_bits;
    float gain_value;
    float it_ms;
    float resolution;
} SensorConfig;

typedef struct {
    float samples[MOVING_AVG_SAMPLES];
    int index;
    int count;
    float sum;
} moving_avg_filter_t;

typedef struct {
    uint16_t raw_counts_history[READING_HISTORY_SIZE];
    int history_index;
    int history_count;
    int readings_since_change;
    bool range_change_pending;
} range_switch_state_t;

/* ======================== Configuration Table ======================== */

static const SensorConfig g_configs[] = {
    /* 2x gain configurations (highest sensitivity) */
    {VEML7700_GAIN_2X, VEML7700_IT_800MS, 2.0f, 800.0f, 0.0036f},
    {VEML7700_GAIN_2X, VEML7700_IT_400MS, 2.0f, 400.0f, 0.0072f},
    {VEML7700_GAIN_2X, VEML7700_IT_200MS, 2.0f, 200.0f, 0.0144f},
    {VEML7700_GAIN_2X, VEML7700_IT_100MS, 2.0f, 100.0f, 0.0288f},
    {VEML7700_GAIN_2X, VEML7700_IT_50MS,  2.0f, 50.0f,  0.0576f},

    /* 1x gain configurations (medium sensitivity) */
    {VEML7700_GAIN_1X, VEML7700_IT_800MS, 1.0f, 800.0f, 0.0072f},
    {VEML7700_GAIN_1X, VEML7700_IT_400MS, 1.0f, 400.0f, 0.0144f},
    {VEML7700_GAIN_1X, VEML7700_IT_200MS, 1.0f, 200.0f, 0.0288f},
    {VEML7700_GAIN_1X, VEML7700_IT_100MS, 1.0f, 100.0f, 0.0576f},
    {VEML7700_GAIN_1X, VEML7700_IT_50MS,  1.0f, 50.0f,  0.1152f},
    {VEML7700_GAIN_1X, VEML7700_IT_25MS,  1.0f, 25.0f,  0.2304f},

    /* 1/4x gain configurations (lower sensitivity) */
    {VEML7700_GAIN_1_4X, VEML7700_IT_800MS, 0.25f, 800.0f, 0.0288f},
    {VEML7700_GAIN_1_4X, VEML7700_IT_400MS, 0.25f, 400.0f, 0.0576f},
    {VEML7700_GAIN_1_4X, VEML7700_IT_200MS, 0.25f, 200.0f, 0.1152f},
    {VEML7700_GAIN_1_4X, VEML7700_IT_100MS, 0.25f, 100.0f, 0.2304f},

    /* 1/8x gain configurations (lowest sensitivity, highest lux range) */
    {VEML7700_GAIN_1_8X, VEML7700_IT_800MS, 0.125f, 800.0f, 0.0576f},
    {VEML7700_GAIN_1_8X, VEML7700_IT_400MS, 0.125f, 400.0f, 0.1152f},
    {VEML7700_GAIN_1_8X, VEML7700_IT_200MS, 0.125f, 200.0f, 0.2304f},
};

#define NUM_CONFIGS (sizeof(g_configs) / sizeof(g_configs[0]))

/* ======================== Static State ======================== */

static moving_avg_filter_t lux_filter = {0};
static range_switch_state_t range_state = {0};
static int current_config_idx = 8;  // Start with 1x gain, 100ms integration
static TickType_t last_range_switch_time = 0;
static bool initialized = false;

/* ======================== Helper Functions ======================== */

static uint16_t sensor_config_get_config_word(const SensorConfig* config) {
    return config->gain_bits | config->it_bits | VEML7700_POWER_ON;
}

static const char* sensor_config_gain_str(const SensorConfig* config) {
    if (config->gain_bits == VEML7700_GAIN_1X) return "1x";
    if (config->gain_bits == VEML7700_GAIN_2X) return "2x";
    if (config->gain_bits == VEML7700_GAIN_1_4X) return "1/4x";
    if (config->gain_bits == VEML7700_GAIN_1_8X) return "1/8x";
    return "?";
}

static const char* sensor_config_it_str(const SensorConfig* config) {
    if (config->it_bits == VEML7700_IT_25MS) return "25ms";
    if (config->it_bits == VEML7700_IT_50MS) return "50ms";
    if (config->it_bits == VEML7700_IT_100MS) return "100ms";
    if (config->it_bits == VEML7700_IT_200MS) return "200ms";
    if (config->it_bits == VEML7700_IT_400MS) return "400ms";
    if (config->it_bits == VEML7700_IT_800MS) return "800ms";
    return "?";
}

/* ======================== Calibration Functions ======================== */

static float calibrate_lux(float raw_lux) {
    /* Piecewise linear calibration for Adafruit VEML7700 with diffuser dome */
    if (raw_lux < 10.0f) {
        return raw_lux * 0.85f;
    } else if (raw_lux < 100.0f) {
        return raw_lux * 0.92f;
    } else if (raw_lux < 500.0f) {
        return raw_lux * 1.05f;
    } else if (raw_lux < 1000.0f) {
        return raw_lux * 1.12f;
    } else {
        return raw_lux * 1.18f;
    }
}

/* ======================== Moving Average Filter ======================== */

static void moving_avg_init(moving_avg_filter_t *filter) {
    memset(filter, 0, sizeof(moving_avg_filter_t));
}

static float moving_avg_update(moving_avg_filter_t *filter, float new_value) {
    if (filter->count == MOVING_AVG_SAMPLES) {
        filter->sum -= filter->samples[filter->index];
    } else {
        filter->count++;
    }

    filter->samples[filter->index] = new_value;
    filter->sum += new_value;
    filter->index = (filter->index + 1) % MOVING_AVG_SAMPLES;

    return filter->sum / filter->count;
}

/* ======================== I2C Communication ======================== */

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
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t veml7700_write_reg(uint8_t reg, uint16_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VEML7700_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data & 0xFF, true);
    i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write error: %s", esp_err_to_name(ret));
    }

    return ret;
}

static esp_err_t veml7700_read_reg(uint8_t reg, uint16_t *data) {
    uint8_t data_rd[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VEML7700_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VEML7700_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data_rd[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data_rd[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *data = (data_rd[1] << 8) | data_rd[0];
    } else {
        ESP_LOGE(TAG, "I2C read error: %s", esp_err_to_name(ret));
    }

    return ret;
}

static esp_err_t veml7700_read_raw_counts(uint16_t *counts) {
    return veml7700_read_reg(VEML7700_REG_ALS, counts);
}

static esp_err_t veml7700_set_config(int config_idx) {
    if (config_idx < 0 || config_idx >= NUM_CONFIGS) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = veml7700_write_reg(VEML7700_REG_CONF,
                                        sensor_config_get_config_word(&g_configs[config_idx]));
    if (ret == ESP_OK) {
        current_config_idx = config_idx;
        last_range_switch_time = xTaskGetTickCount();

        /* Reset filter and range state after config change */
        moving_avg_init(&lux_filter);
        range_state.readings_since_change = 0;
        range_state.history_count = 0;
        range_state.history_index = 0;

        /* Verify configuration */
        uint16_t readback_config;
        if (veml7700_read_reg(VEML7700_REG_CONF, &readback_config) == ESP_OK) {
            ESP_LOGI(TAG, "Config %d: %s gain, %s IT, written=0x%04X, read=0x%04X",
                     config_idx,
                     sensor_config_gain_str(&g_configs[config_idx]),
                     sensor_config_it_str(&g_configs[config_idx]),
                     sensor_config_get_config_word(&g_configs[config_idx]),
                     readback_config);
        }
    }
    return ret;
}

/* ======================== Auto-Ranging Logic ======================== */

static void update_reading_history(uint16_t raw_counts) {
    range_state.raw_counts_history[range_state.history_index] = raw_counts;
    range_state.history_index = (range_state.history_index + 1) % READING_HISTORY_SIZE;

    if (range_state.history_count < READING_HISTORY_SIZE) {
        range_state.history_count++;
    }

    range_state.readings_since_change++;
}

static bool readings_consistently_high(void) {
    if (range_state.history_count < CONSISTENCY_READINGS) return false;

    int check_count = (range_state.history_count < CONSISTENCY_READINGS) ?
                      range_state.history_count : CONSISTENCY_READINGS;

    for (int i = 0; i < check_count; i++) {
        int idx = (range_state.history_index - 1 - i + READING_HISTORY_SIZE) % READING_HISTORY_SIZE;
        if (range_state.raw_counts_history[idx] <= OPTIMAL_HIGH) {
            return false;
        }
    }
    return true;
}

static bool readings_consistently_low(void) {
    if (range_state.history_count < CONSISTENCY_READINGS) return false;

    int check_count = (range_state.history_count < CONSISTENCY_READINGS) ?
                      range_state.history_count : CONSISTENCY_READINGS;

    for (int i = 0; i < check_count; i++) {
        int idx = (range_state.history_index - 1 - i + READING_HISTORY_SIZE) % READING_HISTORY_SIZE;
        if (range_state.raw_counts_history[idx] >= OPTIMAL_LOW) {
            return false;
        }
    }
    return true;
}

static int determine_optimal_config(uint16_t raw_counts) {
    update_reading_history(raw_counts);

    /* Don't adjust immediately after configuration change */
    if (range_state.readings_since_change < SETTLE_READINGS_REQUIRED) {
        ESP_LOGD(TAG, "Settling after config change (%d/%d readings)",
                 range_state.readings_since_change, SETTLE_READINGS_REQUIRED);
        return current_config_idx;
    }

    /* Determine if adjustment needed - SINGLE STEP ONLY */
    int adjustment = 0;
    const char* reason = "";

    /* Critical thresholds - immediate action */
    if (raw_counts >= SATURATED_HIGH || raw_counts >= 65000) {
        adjustment = 1;  // Less sensitive
        reason = raw_counts >= 64000 ? "SATURATED" : "TOO_HIGH";

    } else if (raw_counts < TOO_LOW && raw_counts > 0) {
        adjustment = -1;  // More sensitive
        reason = raw_counts < 500 ? "VERY_LOW" : "TOO_LOW";

    } else if (raw_counts >= OPTIMAL_LOW && raw_counts <= OPTIMAL_HIGH) {
        /* In optimal range - no change */
        ESP_LOGV(TAG, "Raw counts %d in optimal range (%d-%d)",
                 raw_counts, OPTIMAL_LOW, OPTIMAL_HIGH);
        return current_config_idx;

    } else {
        /* Use consistency check */
        if (raw_counts > OPTIMAL_HIGH && raw_counts < SATURATED_HIGH) {
            if (readings_consistently_high()) {
                adjustment = 1;
                reason = "CONSISTENTLY_HIGH";
            }
        } else if (raw_counts < OPTIMAL_LOW && raw_counts >= TOO_LOW) {
            if (readings_consistently_low()) {
                adjustment = -1;
                reason = "CONSISTENTLY_LOW";
            }
        }
    }

    /* Apply single-step adjustment */
    if (adjustment != 0) {
        int new_config_idx = current_config_idx + adjustment;

        /* Clamp to valid range */
        if (new_config_idx < 0) new_config_idx = 0;
        if (new_config_idx >= NUM_CONFIGS) new_config_idx = NUM_CONFIGS - 1;

        if (new_config_idx != current_config_idx) {
            ESP_LOGI(TAG, "Config switch: %d→%d, reason=%s (raw=%d)",
                     current_config_idx, new_config_idx, reason, raw_counts);
            return new_config_idx;
        }
    }

    return current_config_idx;
}

/* ======================== Public API ======================== */

esp_err_t veml7700_init(void) {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* Initialize I2C */
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        return ret;
    }

    /* Initialize filter and state */
    moving_avg_init(&lux_filter);
    memset(&range_state, 0, sizeof(range_state));

    /* Set initial configuration */
    ret = veml7700_set_config(current_config_idx);
    if (ret == ESP_OK) {
        initialized = true;
        ESP_LOGI(TAG, "Initialized successfully");
    }

    return ret;
}

esp_err_t veml7700_read_lux(float *lux) {
    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (lux == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read raw counts */
    uint16_t raw_counts;
    esp_err_t ret = veml7700_read_raw_counts(&raw_counts);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Apply auto-ranging */
    int optimal_config = determine_optimal_config(raw_counts);

    /* Switch config if needed */
    if (optimal_config != current_config_idx) {
        ret = veml7700_set_config(optimal_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Config switch failed, continuing with current");
        } else {
            /* Wait for sensor to settle */
            float settle_time = g_configs[optimal_config].it_ms * 3.0f + 200;
            vTaskDelay(pdMS_TO_TICKS((int)settle_time));

            /* Read fresh counts and discard stale readings */
            for (int i = 0; i < 3; i++) {
                ret = veml7700_read_raw_counts(&raw_counts);
                if (ret != ESP_OK) {
                    return ret;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            update_reading_history(raw_counts);
        }
    }

    /* Convert to lux */
    float raw_lux = raw_counts * g_configs[current_config_idx].resolution;

    /* Apply moving average filter */
    float filtered_lux = moving_avg_update(&lux_filter, raw_lux);

    /* Apply diffuser calibration */
    const float DIFFUSER_CALIBRATION = 1.96f;
    float calibrated_lux = filtered_lux * DIFFUSER_CALIBRATION;

    /* Apply piecewise calibration */
    *lux = calibrate_lux(calibrated_lux);
    if (*lux < 0.0f) *lux = 0.0f;

    ESP_LOGD(TAG, "Read: raw_counts=%d, raw_lux=%.1f, final_lux=%.1f, config=%d",
             raw_counts, raw_lux, *lux, current_config_idx);

    return ESP_OK;
}

uint8_t veml7700_get_config_idx(void) {
    return (uint8_t)current_config_idx;
}

const char* veml7700_get_status_string(void) {
    if (!initialized) {
        return "Not Initialized";
    }
    return "OK";
}
