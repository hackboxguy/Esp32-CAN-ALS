/* main.c - ESP32-C6 CAN Lux Sensor with VEML7700 Auto-Ranging */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <driver/twai.h>
#include <driver/i2c.h>

#define CAN_TX_GPIO_NUM         GPIO_NUM_4
#define CAN_RX_GPIO_NUM         GPIO_NUM_5
#define I2C_SDA_GPIO_NUM        GPIO_NUM_6
#define I2C_SCL_GPIO_NUM        GPIO_NUM_7
#define CAN_TX_TASK_PRIO        8
#define CAN_RX_TASK_PRIO        9
#define CAN_CTRL_TSK_PRIO       10
#define CAN_DATA_PERIOD_MS      1000
#define CAN_NO_WAIT             0
#define CAN_WAIT_FOREVER        portMAX_DELAY
#define ID_MASTER_STOP_CMD      0x0A0
#define ID_MASTER_START_CMD     0x0A1
#define ID_MASTER_DATA          0x0A2

// VEML7700 I2C Configuration
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define VEML7700_I2C_ADDR       0x10
#define VEML7700_REG_CONF       0x00
#define VEML7700_REG_ALS        0x04

// VEML7700 Configuration Bits (aligned with datasheet)
#define VEML7700_GAIN_1X        (0x00 << 11)  // 0x0000
#define VEML7700_GAIN_2X        (0x01 << 11)  // 0x0800
#define VEML7700_GAIN_1_4X      (0x02 << 11)  // 0x1000
#define VEML7700_GAIN_1_8X      (0x03 << 11)  // 0x1800

#define VEML7700_IT_25MS        0x0300
#define VEML7700_IT_50MS        0x0200
#define VEML7700_IT_100MS       0x0000
#define VEML7700_IT_200MS       0x0040
#define VEML7700_IT_400MS       0x0080
#define VEML7700_IT_800MS       0x00C0

#define VEML7700_POWER_ON       0x0000
#define VEML7700_POWER_OFF      0x0001

// Auto-ranging parameters
#define SATURATED_HIGH          60000   // Near ADC saturation
#define OPTIMAL_HIGH            40000   // Upper optimal range
#define OPTIMAL_LOW             10000   // Lower optimal range
#define TOO_LOW                 2000    // Force sensitivity increase

// History tracking for stability
#define READING_HISTORY_SIZE    5
#define SETTLE_READINGS_REQUIRED 3
#define CONSISTENCY_READINGS    3

// Moving average filter settings
#define MOVING_AVG_SAMPLES      8

static const char *TAG = "CAN_LUX_SENSOR";

static SemaphoreHandle_t done_sem;
static SemaphoreHandle_t start_sem;

// Add this function to help debug the calibration
static void debug_calibration(float raw_lux, float calibrated_lux, const char* range) {
    ESP_LOGI(TAG, "Calibration debug: raw=%.1f, calibrated=%.1f, range=%s", 
             raw_lux, calibrated_lux, range);
}

// calibration function(old but no so precise)
static float calibrate_lux_old(float raw_lux) {
    // Linear calibration based on your measurements: y = 0.0004x + 1.7
    // where x is the raw lux value and y is the calibration factor
    float calibration_factor = 0.0004f * raw_lux + 1.7f;
    return raw_lux * calibration_factor;
}
static float calibrate_lux(float raw_lux) {
    // Revised calibration based on your measurements
    // Uses a polynomial fit for better accuracy across the range
    if (raw_lux < 10.0f) {
        return raw_lux * 0.85f;  // Adjustment for very low light
    } else if (raw_lux < 100.0f) {
        return raw_lux * 0.92f;  // Adjustment for low light
    } else if (raw_lux < 500.0f) {
        return raw_lux * 1.05f;  // Adjustment for medium light
    } else if (raw_lux < 1000.0f) {
        return raw_lux * 1.12f;  // Adjustment for high light
    } else {
        return raw_lux * 1.18f;  // Adjustment for very high light
    }
}

// VEML7700 Configuration Structure
typedef struct {
    uint16_t gain_bits;
    uint16_t it_bits;
    float gain_value;
    float it_ms;
    float resolution;  // counts per lux (approximate)
} SensorConfig;

// Helper functions for SensorConfig
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

// Configuration table based on C++ code (11 configurations)
static const SensorConfig g_configs[] = {
    // 2x gain configurations
    {VEML7700_GAIN_2X, VEML7700_IT_800MS, 2.0f, 800.0f, 0.0036f},
    {VEML7700_GAIN_2X, VEML7700_IT_400MS, 2.0f, 400.0f, 0.0072f},
    {VEML7700_GAIN_2X, VEML7700_IT_200MS, 2.0f, 200.0f, 0.0144f},
    {VEML7700_GAIN_2X, VEML7700_IT_100MS, 2.0f, 100.0f, 0.0288f},
    {VEML7700_GAIN_2X, VEML7700_IT_50MS, 2.0f, 50.0f, 0.0576f},

    // 1x gain configurations
    {VEML7700_GAIN_1X, VEML7700_IT_800MS, 1.0f, 800.0f, 0.0072f},
    {VEML7700_GAIN_1X, VEML7700_IT_400MS, 1.0f, 400.0f, 0.0144f},
    {VEML7700_GAIN_1X, VEML7700_IT_200MS, 1.0f, 200.0f, 0.0288f},
    {VEML7700_GAIN_1X, VEML7700_IT_100MS, 1.0f, 100.0f, 0.0576f},
    {VEML7700_GAIN_1X, VEML7700_IT_50MS, 1.0f, 50.0f, 0.1152f},
    {VEML7700_GAIN_1X, VEML7700_IT_25MS, 1.0f, 25.0f, 0.2304f},

    // 1/4x gain configurations
    {VEML7700_GAIN_1_4X, VEML7700_IT_800MS, 0.25f, 800.0f, 0.0288f},
    {VEML7700_GAIN_1_4X, VEML7700_IT_400MS, 0.25f, 400.0f, 0.0576f},
    {VEML7700_GAIN_1_4X, VEML7700_IT_200MS, 0.25f, 200.0f, 0.1152f},
    {VEML7700_GAIN_1_4X, VEML7700_IT_100MS, 0.25f, 100.0f, 0.2304f},

    // 1/8x gain configurations
    {VEML7700_GAIN_1_8X, VEML7700_IT_800MS, 0.125f, 800.0f, 0.0576f},
    {VEML7700_GAIN_1_8X, VEML7700_IT_400MS, 0.125f, 400.0f, 0.1152f},
    {VEML7700_GAIN_1_8X, VEML7700_IT_200MS, 0.125f, 200.0f, 0.2304f},
};

#define NUM_CONFIGS (sizeof(g_configs) / sizeof(g_configs[0]))

// Moving average filter structure
typedef struct {
    float samples[MOVING_AVG_SAMPLES];
    int index;
    int count;
    float sum;
} moving_avg_filter_t;

// Intelligent range switching state
typedef struct {
    uint16_t raw_counts_history[READING_HISTORY_SIZE];
    int history_index;
    int history_count;
    int readings_since_change;
    bool range_change_pending;
} range_switch_state_t;

// State tracking
static moving_avg_filter_t lux_filter = {0};
static range_switch_state_t range_state = {0};
static int current_config_idx = 7;  // Start with 1x gain, 100ms integration
static TickType_t last_range_switch_time = 0;

/* --------------------------- Moving Average Filter ------------------------ */

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

/* --------------------------- VEML7700 Functions --------------------------- */

static esp_err_t i2c_master_init(void)
{
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
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t veml7700_write_reg(uint8_t reg, uint16_t data)
{
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

static esp_err_t veml7700_read_reg(uint8_t reg, uint16_t *data)
{
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

static esp_err_t veml7700_read_raw_counts(uint16_t *counts)
{
    return veml7700_read_reg(VEML7700_REG_ALS, counts);
}

static esp_err_t veml7700_set_config(int config_idx)
{
    if (config_idx < 0 || config_idx >= NUM_CONFIGS) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = veml7700_write_reg(VEML7700_REG_CONF, sensor_config_get_config_word(&g_configs[config_idx]));
    if (ret == ESP_OK) {
        current_config_idx = config_idx;
        last_range_switch_time = xTaskGetTickCount();

        // Reset moving average filter and range state after config change
        moving_avg_init(&lux_filter);
        range_state.readings_since_change = 0;
        range_state.history_count = 0;
        range_state.history_index = 0;

        // Verify configuration was written correctly
        uint16_t readback_config;
        if (veml7700_read_reg(VEML7700_REG_CONF, &readback_config) == ESP_OK) {
            ESP_LOGI(TAG, "Config %d: %s gain, %s integration, config=0x%04X, readback=0x%04X",
                     config_idx,
                     sensor_config_gain_str(&g_configs[config_idx]),
                     sensor_config_it_str(&g_configs[config_idx]),
                     sensor_config_get_config_word(&g_configs[config_idx]),
                     readback_config);
        }
    }
    return ret;
}

static esp_err_t veml7700_init(void)
{
    moving_avg_init(&lux_filter);
    memset(&range_state, 0, sizeof(range_state));
    return veml7700_set_config(current_config_idx);
}

/* --------------------------- Intelligent Auto-Ranging -------------------- */

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
    // Update history
    update_reading_history(raw_counts);

    // Don't adjust immediately after a configuration change
    if (range_state.readings_since_change < SETTLE_READINGS_REQUIRED) {
        ESP_LOGD(TAG, "Settling after config change (%d/%d readings)",
                 range_state.readings_since_change, SETTLE_READINGS_REQUIRED);
        return current_config_idx;
    }

    // Determine if config adjustment is needed - SINGLE STEP ONLY
    int adjustment = 0;
    const char* reason = "";

    // Critical thresholds - immediate action required
    if (raw_counts >= SATURATED_HIGH || raw_counts >= 65000) {
        // Near saturation - decrease sensitivity immediately
        adjustment = 1;  // Move to less sensitive config
        reason = raw_counts >= 64000 ? "SATURATED" : "TOO_HIGH";

    } else if (raw_counts < TOO_LOW && raw_counts > 0) {
        // Too low - increase sensitivity immediately
        adjustment = -1;  // Move to more sensitive config
        reason = raw_counts < 500 ? "VERY_LOW" : "TOO_LOW";

    } else if (raw_counts >= OPTIMAL_LOW && raw_counts <= OPTIMAL_HIGH) {
        // In optimal range - no change needed
        ESP_LOGV(TAG, "Raw counts %d in optimal range (%d-%d)",
                 raw_counts, OPTIMAL_LOW, OPTIMAL_HIGH);
        return current_config_idx;

    } else {
        // In acceptable but not optimal range - use consistency check
        if (raw_counts > OPTIMAL_HIGH && raw_counts < SATURATED_HIGH) {
            // Check if consistently high
            if (readings_consistently_high()) {
                adjustment = 1;  // Less sensitive
                reason = "CONSISTENTLY_HIGH";
            }
        } else if (raw_counts < OPTIMAL_LOW && raw_counts >= TOO_LOW) {
            // Check if consistently low
            if (readings_consistently_low()) {
                adjustment = -1;  // More sensitive
                reason = "CONSISTENTLY_LOW";
            }
        }
    }

    // Apply single-step adjustment
    if (adjustment != 0) {
        int new_config_idx = current_config_idx + adjustment;

        // Clamp to valid range
        if (new_config_idx < 0) new_config_idx = 0;
        if (new_config_idx >= NUM_CONFIGS) new_config_idx = NUM_CONFIGS - 1;

        if (new_config_idx != current_config_idx) {
            ESP_LOGI(TAG, "Config switch: %d->%d, reason=%s (raw_counts=%d)",
                     current_config_idx, new_config_idx, reason, raw_counts);

            return new_config_idx;
        }
    }

    return current_config_idx;
}

static esp_err_t veml7700_read_lux_with_auto_ranging(float *stable_lux) {
    uint16_t raw_counts;
    esp_err_t ret = veml7700_read_raw_counts(&raw_counts);

    if (ret != ESP_OK) {
        return ret;
    }

    // Apply intelligent auto-ranging
    int optimal_config = determine_optimal_config(raw_counts);

    // Switch config if needed
    if (optimal_config != current_config_idx) {
        ret = veml7700_set_config(optimal_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Config switch failed, continuing with current config");
        } else {
            // Wait for sensor to settle after config change
            // Integration time * 3 + settling time, based on Linux implementation
            float settle_time = g_configs[optimal_config].it_ms * 3.0f + 200;
            vTaskDelay(pdMS_TO_TICKS((int)settle_time));

            // Read fresh counts with new config and discard stale readings
            for (int i = 0; i < 3; i++) {
                ret = veml7700_read_raw_counts(&raw_counts);
                if (ret != ESP_OK) {
                    return ret;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            // Update history with fresh reading
            update_reading_history(raw_counts);
        }
    }

    // Convert raw counts to lux
    float raw_lux = raw_counts * g_configs[current_config_idx].resolution;

    // Apply moving average filter for additional stability
    float filtered_lux = moving_avg_update(&lux_filter, raw_lux);

    // Apply calibration factor for Adafruit board with diffuser dome
    const float DIFFUSER_CALIBRATION = 1.96f;//1.13f;
    float calibrated_lux = filtered_lux * DIFFUSER_CALIBRATION;
    //*stable_lux = filtered_lux * DIFFUSER_CALIBRATION;
    *stable_lux = calibrate_lux(calibrated_lux);//extra calibration
    if (*stable_lux < 0.0) *stable_lux = 0.0;

    // Debug logging
    const char* range = "unknown";
    if (calibrated_lux < 50.0f) range = "low";
    else if (calibrated_lux < 500.0f) range = "low-medium";
    else if (calibrated_lux < 1000.0f) range = "medium-high";
    else range = "high";
    debug_calibration(calibrated_lux, *stable_lux, range);


    //ESP_LOGV(TAG, "Sensor: raw_counts=%d, raw_lux=%.1f, filtered_lux=%.1f, final_lux=%.1f",
    //         raw_counts, raw_lux, filtered_lux, *stable_lux);
    //ESP_LOGV(TAG, "Sensor: raw_counts=%d, raw_lux=%.1f, filtered_lux=%.1f, calibrated_lux=%.1f, final_lux=%.1f",
    //     raw_counts, raw_lux, filtered_lux, calibrated_lux, *stable_lux);

    return ESP_OK;
}

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    twai_message_t rx_msg;

    while (1) {
        if (twai_receive(&rx_msg, CAN_WAIT_FOREVER) == ESP_OK) {
            if (!(rx_msg.flags & TWAI_MSG_FLAG_RTR)) {
                if (rx_msg.identifier == ID_MASTER_STOP_CMD) {
                    xSemaphoreGive(done_sem);
                } else if (rx_msg.identifier == ID_MASTER_START_CMD) {
                    xSemaphoreGive(start_sem);
                }
            }
        }
    }
}

static void twai_transmit_task(void *arg)
{
    twai_message_t tx_msg;
    float lux_value = 0.0;
    uint16_t lux_int = 0;
    uint8_t sequence_counter = 0;
    uint8_t sensor_status = 0x00;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    tx_msg.identifier = ID_MASTER_DATA;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;
    tx_msg.data_length_code = 8;

    while (1) {
        if (xSemaphoreTake(start_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        sequence_counter = 0;
        xLastWakeTime = xTaskGetTickCount();

        while (1) {
            if (xSemaphoreTake(done_sem, 0) == pdTRUE) {
                break;
            }

            // Read lux with auto-ranging
            esp_err_t lux_result = veml7700_read_lux_with_auto_ranging(&lux_value);
            if (lux_result == ESP_OK) {
                lux_int = (uint16_t)(lux_value > 65535.0 ? 65535 : lux_value);
                sensor_status = 0x00;
            } else {
                lux_int = 0xFFFF;
                sensor_status = 0x01;
            }

            // Build message
            memset(tx_msg.data, 0, 8);
            tx_msg.data[0] = lux_int & 0xFF;
            tx_msg.data[1] = (lux_int >> 8) & 0xFF;
            tx_msg.data[2] = sensor_status;
            tx_msg.data[3] = sequence_counter;
            tx_msg.data[4] = current_config_idx;
            tx_msg.data[5] = 0x00;

            // Checksum
            uint16_t checksum = 0;
            for (int i = 0; i < 6; i++) {
                checksum += tx_msg.data[i];
            }
            tx_msg.data[6] = checksum & 0xFF;
            tx_msg.data[7] = (checksum >> 8) & 0xFF;

            twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
            sequence_counter++;

            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_DATA_PERIOD_MS));
        }
    }

    vTaskDelete(NULL);
}

static void twai_control_task(void *arg)
{
    twai_message_t tx_msg;

    tx_msg.identifier = ID_MASTER_START_CMD;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;
    tx_msg.data_length_code = 0;

    if (twai_transmit(&tx_msg, CAN_WAIT_FOREVER) == ESP_OK) {
        xSemaphoreGive(start_sem);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    // Create semaphores
    done_sem = xSemaphoreCreateBinary();
    start_sem = xSemaphoreCreateBinary();

    if (done_sem == NULL || start_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return;
    }

    // Initialize I2C and VEML7700
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");

    ESP_ERROR_CHECK(veml7700_init());
    ESP_LOGI(TAG, "VEML7700 initialized with %s gain, %s integration",
             sensor_config_gain_str(&g_configs[current_config_idx]),
             sensor_config_it_str(&g_configs[current_config_idx]));

    // Configure TWAI
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = CAN_TX_GPIO_NUM,
        .rx_io = CAN_RX_GPIO_NUM,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 10,
        .rx_queue_len = 10,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0
    };

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI initialized at 500kbps");

    // Create tasks
    xTaskCreate(twai_receive_task, "TWAI_RX", 3072, NULL, CAN_RX_TASK_PRIO, NULL);
    xTaskCreate(twai_transmit_task, "TWAI_TX", 4096, NULL, CAN_TX_TASK_PRIO, NULL);
    xTaskCreate(twai_control_task, "TWAI_CTRL", 2048, NULL, CAN_CTRL_TSK_PRIO, NULL);

    ESP_LOGI(TAG, "ESP32-C6 CAN Lux Sensor with VEML7700 Auto-Ranging started");
    ESP_LOGI(TAG, "Auto-ranging thresholds: OPTIMAL(%d-%d counts), TOO_LOW(%d), SATURATED(%d+)",
             OPTIMAL_LOW, OPTIMAL_HIGH, TOO_LOW, SATURATED_HIGH);

    // Keep main task alive with periodic status
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Log status every 10 seconds

        float current_lux;
        if (veml7700_read_lux_with_auto_ranging(&current_lux) == ESP_OK) {
            ESP_LOGI(TAG, "Status: %.1f lux, Config: %d (%s gain, %s integration), Filter samples: %d, Config readings: %d",
                     current_lux, current_config_idx,
                     sensor_config_gain_str(&g_configs[current_config_idx]),
                     sensor_config_it_str(&g_configs[current_config_idx]),
                     lux_filter.count, range_state.readings_since_change);
        }
    }
}
