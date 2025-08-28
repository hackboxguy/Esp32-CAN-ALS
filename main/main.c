/* main.c - ESP32-C6 CAN Lux Sensor with Calibration */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <driver/twai.h>
#include <driver/i2c.h>
#include <nvs_flash.h>
#include <nvs.h>

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
#define ID_CALIBRATION_CMD      0x0A3
#define ID_CALIBRATION_RESP     0x0A4
#define ID_RAW_SENSOR_DATA      0x0A5

// VEML7700 I2C Configuration
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define VEML7700_I2C_ADDR       0x10
#define VEML7700_REG_CONF       0x00
#define VEML7700_REG_ALS        0x04

    // Calibration commands
#define CAL_CMD_ENTER           0x01
#define CAL_CMD_SET_REFERENCE   0x02
#define CAL_CMD_SAVE            0x03
#define CAL_CMD_EXIT            0x04
#define CAL_CMD_GET_OFFSET      0x05
#define CAL_CMD_RESET           0x06

// NVS key for calibration
#define NVS_NAMESPACE           "lux_sensor"
#define NVS_CALIBRATION_KEY     "cal_offset"

typedef enum {
    CAL_STATE_NORMAL = 0,
    CAL_STATE_ACTIVE = 1,
    CAL_STATE_WAITING_REF = 2
} calibration_state_t;

static const char *TAG = "CAN_LUX_SENSOR";

static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t done_sem;
static SemaphoreHandle_t start_sem;

// Calibration variables
static float calibration_offset = 0.0;
static calibration_state_t calibration_state = CAL_STATE_NORMAL;
static float reference_lux = 0.0;

/* --------------------------- NVS Functions -------------------------------- */

static esp_err_t save_calibration_offset(float offset)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(nvs_handle, NVS_CALIBRATION_KEY, &offset, sizeof(float));
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);
    return err;
}

static esp_err_t load_calibration_offset(float *offset)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        *offset = 0.0;  // Default to no offset
        return err;
    }

    size_t required_size = sizeof(float);
    err = nvs_get_blob(nvs_handle, NVS_CALIBRATION_KEY, offset, &required_size);
    if (err != ESP_OK) {
        *offset = 0.0;  // Default to no offset
    }

    nvs_close(nvs_handle);
    return err;
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
    i2c_master_write_byte(cmd, data & 0xFF, true);        // Low byte
    i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true); // High byte
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
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
    }

    return ret;
}

static esp_err_t veml7700_init(void)
{
    // Configure VEML7700: ALS integration time 100ms, gain 1, enable ALS
    uint16_t config = 0x0000;  // Default configuration
    return veml7700_write_reg(VEML7700_REG_CONF, config);
}

static esp_err_t veml7700_read_lux(float *lux)
{
    uint16_t als_data;
    esp_err_t ret = veml7700_read_reg(VEML7700_REG_ALS, &als_data);

    if (ret == ESP_OK) {
        // Convert raw ALS data to lux and apply calibration offset
        float raw_lux = als_data * 0.0036;
        *lux = raw_lux + calibration_offset;

        // Ensure lux value is not negative
        if (*lux < 0.0) {
            *lux = 0.0;
        }
    }

    return ret;
}

static esp_err_t veml7700_read_raw_lux(float *raw_lux)
{
    uint16_t als_data;
    esp_err_t ret = veml7700_read_reg(VEML7700_REG_ALS, &als_data);

    if (ret == ESP_OK) {
        // Return raw lux without calibration offset
        *raw_lux = als_data * 0.0036;
    }

    return ret;
}

/* --------------------------- Calibration Functions ----------------------- */

static void send_calibration_response(uint8_t command, uint8_t status, const uint8_t *data, size_t data_len)
{
    twai_message_t response_msg;
    response_msg.identifier = ID_CALIBRATION_RESP;
    response_msg.flags = TWAI_MSG_FLAG_NONE;
    response_msg.data_length_code = 8;

    memset(response_msg.data, 0, 8);
    response_msg.data[0] = command;  // Echo command
    response_msg.data[1] = status;   // 0x00=OK, 0x01=Error

    // Copy data if provided
    if (data && data_len > 0) {
        size_t copy_len = (data_len > 6) ? 6 : data_len;
        memcpy(&response_msg.data[2], data, copy_len);
    }

    twai_transmit(&response_msg, pdMS_TO_TICKS(1000));
}

static void send_raw_sensor_data(float raw_lux)
{
    twai_message_t raw_msg;
    raw_msg.identifier = ID_RAW_SENSOR_DATA;
    raw_msg.flags = TWAI_MSG_FLAG_NONE;
    raw_msg.data_length_code = 8;

    uint16_t raw_lux_int = (uint16_t)(raw_lux > 65535.0 ? 65535 : raw_lux);

    memset(raw_msg.data, 0, 8);
    raw_msg.data[0] = raw_lux_int & 0xFF;
    raw_msg.data[1] = (raw_lux_int >> 8) & 0xFF;
    raw_msg.data[2] = 0xAA;  // Raw data marker
    raw_msg.data[3] = 0x55;  // Raw data marker

    twai_transmit(&raw_msg, pdMS_TO_TICKS(1000));
}

static void handle_calibration_command(uint8_t command, const uint8_t *data)
{
    switch (command) {
        case CAL_CMD_ENTER:
            calibration_state = CAL_STATE_ACTIVE;
            send_calibration_response(command, 0x00, NULL, 0);  // OK
            break;

        case CAL_CMD_SET_REFERENCE:
            if (calibration_state == CAL_STATE_ACTIVE) {
                // Extract reference lux value (16-bit)
                reference_lux = (float)(data[0] | (data[1] << 8));

                // Read current raw sensor value
                float current_raw_lux;
                if (veml7700_read_raw_lux(&current_raw_lux) == ESP_OK) {
                    // Calculate offset: reference - current_raw
                    calibration_offset = reference_lux - current_raw_lux;
                    calibration_state = CAL_STATE_WAITING_REF;
                    send_calibration_response(command, 0x00, NULL, 0);  // OK
                } else {
                    send_calibration_response(command, 0x01, NULL, 0);  // Error
                }
            } else {
                send_calibration_response(command, 0x01, NULL, 0);  // Error - not in calibration mode
            }
            break;

        case CAL_CMD_SAVE:
            if (calibration_state == CAL_STATE_WAITING_REF) {
                if (save_calibration_offset(calibration_offset) == ESP_OK) {
                    send_calibration_response(command, 0x00, NULL, 0);  // OK
                } else {
                    send_calibration_response(command, 0x01, NULL, 0);  // Error saving
                }
            } else {
                send_calibration_response(command, 0x01, NULL, 0);  // Error - no calibration calculated
            }
            break;

        case CAL_CMD_EXIT:
            calibration_state = CAL_STATE_NORMAL;
            send_calibration_response(command, 0x00, NULL, 0);  // OK
            break;

        case CAL_CMD_GET_OFFSET:
            {
                uint8_t offset_data[4];
                memcpy(offset_data, &calibration_offset, sizeof(float));
                send_calibration_response(command, 0x00, offset_data, 4);
            }
            break;

        case CAL_CMD_RESET:
            // Reset calibration offset to zero
            calibration_offset = 0.0;

            // Save the reset offset to NVS
            if (save_calibration_offset(calibration_offset) == ESP_OK) {
                send_calibration_response(command, 0x00, NULL, 0);  // OK
            } else {
                send_calibration_response(command, 0x01, NULL, 0);  // Error saving
            }
            break;

        default:
            send_calibration_response(command, 0x01, NULL, 0);  // Unknown command
            break;
    }
}

/* --------------------------- Message Functions ---------------------------- */

static bool validate_lux_message(const twai_message_t *msg)
{
    if (msg->data_length_code != 8) {
        return false;
    }

    // Calculate expected checksum
    uint16_t expected_checksum = 0;
    for (int i = 0; i < 6; i++) {
        expected_checksum += msg->data[i];
    }

    // Extract received checksum
    uint16_t received_checksum = msg->data[6] | (msg->data[7] << 8);

    return (expected_checksum == received_checksum);
}

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    twai_message_t rx_msg;

    while (1) {
        if (twai_receive(&rx_msg, CAN_WAIT_FOREVER) == ESP_OK) {
            if (!(rx_msg.flags & TWAI_MSG_FLAG_RTR)) {
                // Handle received commands
                if (rx_msg.identifier == ID_MASTER_STOP_CMD) {
                    xSemaphoreGive(done_sem);
                } else if (rx_msg.identifier == ID_MASTER_START_CMD) {
                    xSemaphoreGive(start_sem);
                } else if (rx_msg.identifier == ID_CALIBRATION_CMD) {
                    // Handle calibration commands
                    if (rx_msg.data_length_code >= 1) {
                        uint8_t command = rx_msg.data[0];
                        handle_calibration_command(command, &rx_msg.data[1]);
                    }
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
    uint8_t sensor_status = 0x00;  // 0x00=OK, 0x01=Error, 0x02=Calibrating
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Configure TX message
    tx_msg.identifier = ID_MASTER_DATA;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;
    tx_msg.data_length_code = 8;

    while (1) {
        // Wait for start command (initially or after stop)
        if (xSemaphoreTake(start_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Reset sequence counter on start
        sequence_counter = 0;
        xLastWakeTime = xTaskGetTickCount();

        // Transmission loop
        while (1) {
            // Check for stop command
            if (xSemaphoreTake(done_sem, 0) == pdTRUE) {
                break;
            }

            // Read lux value from VEML7700
            esp_err_t lux_result;

            if (calibration_state == CAL_STATE_ACTIVE) {
                // During calibration, send raw sensor data instead of normal lux messages
                float raw_lux;
                if (veml7700_read_raw_lux(&raw_lux) == ESP_OK) {
                    send_raw_sensor_data(raw_lux);
                }

                // Wait for next transmission
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_DATA_PERIOD_MS));
                continue;
            }

            // Normal operation - read calibrated lux value
            lux_result = veml7700_read_lux(&lux_value);
            if (lux_result == ESP_OK) {
                lux_int = (uint16_t)(lux_value > 65535.0 ? 65535 : lux_value);
                sensor_status = 0x00;  // OK
            } else {
                lux_int = 0xFFFF;      // Error value
                sensor_status = 0x01;  // Error
            }

            // Set sensor status based on calibration state
            if (calibration_state != CAL_STATE_NORMAL) {
                sensor_status = 0x02;  // Calibrating
            }

            // Build structured message
            memset(tx_msg.data, 0, 8);
            tx_msg.data[0] = lux_int & 0xFF;        // Lux low byte
            tx_msg.data[1] = (lux_int >> 8) & 0xFF; // Lux high byte
            tx_msg.data[2] = sensor_status;         // Status byte
            tx_msg.data[3] = sequence_counter;      // Sequence counter
            tx_msg.data[4] = 0x00;                  // Reserved
            tx_msg.data[5] = 0x00;                  // Reserved

            // Calculate simple 16-bit checksum (sum of first 6 bytes)
            uint16_t checksum = 0;
            for (int i = 0; i < 6; i++) {
                checksum += tx_msg.data[i];
            }
            tx_msg.data[6] = checksum & 0xFF;       // Checksum low byte
            tx_msg.data[7] = (checksum >> 8) & 0xFF; // Checksum high byte

            // Transmit message
            twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));

            // Increment sequence counter (0-255, then wrap)
            sequence_counter++;

            // Wait for next transmission
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_DATA_PERIOD_MS));
        }
    }

    vTaskDelete(NULL);
}

static void twai_control_task(void *arg)
{
    twai_message_t tx_msg;

    // Send start command
    tx_msg.identifier = ID_MASTER_START_CMD;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;
    tx_msg.data_length_code = 0;

    if (twai_transmit(&tx_msg, CAN_WAIT_FOREVER) == ESP_OK) {
        // Signal transmit task to start
        xSemaphoreGive(start_sem);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    // Create semaphores
    ctrl_task_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    start_sem = xSemaphoreCreateBinary();

    if (ctrl_task_sem == NULL || done_sem == NULL || start_sem == NULL) {
        return;
    }

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    // Load calibration offset from NVS
    load_calibration_offset(&calibration_offset);

    // Initialize I2C and VEML7700
    if (i2c_master_init() != ESP_OK) {
        return;
    }

    if (veml7700_init() != ESP_OK) {
        return;
    }

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
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return;
    }

    if (twai_start() != ESP_OK) {
        return;
    }

    // Create tasks
    xTaskCreate(twai_receive_task, "TWAI_RX", 3072, NULL, CAN_RX_TASK_PRIO, NULL);
    xTaskCreate(twai_transmit_task, "TWAI_TX", 3072, NULL, CAN_TX_TASK_PRIO, NULL);
    xTaskCreate(twai_control_task, "TWAI_CTRL", 2048, NULL, CAN_CTRL_TSK_PRIO, NULL);

    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
