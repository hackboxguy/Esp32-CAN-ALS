/* main.c - ESP32-C3/C6 Multi-Sensor CAN Node (Refactored) */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <driver/twai.h>

#include "sensor_common.h"
#include "veml7700_driver.h"
#include "can_protocol.h"

/* ======================== Configuration ======================== */

#define CAN_TX_GPIO_NUM         GPIO_NUM_4
#define CAN_RX_GPIO_NUM         GPIO_NUM_5

#define CAN_TX_TASK_PRIO        8
#define CAN_RX_TASK_PRIO        9
#define CAN_CTRL_TSK_PRIO       10
#define SENSOR_POLL_TASK_PRIO   5

#define CAN_DATA_PERIOD_MS      1000
#define SENSOR_QUEUE_DEPTH      20

static const char *TAG = "MULTI_SENSOR";

/* ======================== Global State ======================== */

static SemaphoreHandle_t done_sem;
static SemaphoreHandle_t start_sem;
static QueueHandle_t sensor_queue;

/* ======================== Sensor Poll Task ======================== */

static void sensor_poll_task(void *arg) {
    sensor_data_t msg;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    ESP_LOGI(TAG, "Sensor poll task started");

    while (1) {
        /* Always read and queue - let transmit task control when to send */
        {
            /* Read VEML7700 */
            msg.sensor_id = SENSOR_VEML7700;
            msg.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if (veml7700_read_lux(&msg.data.veml.lux) == ESP_OK) {
                msg.status = SENSOR_STATUS_OK;
                msg.data.veml.config_idx = veml7700_get_config_idx();
                ESP_LOGD(TAG, "Read VEML7700: %.1f lux, config=%d",
                         msg.data.veml.lux, msg.data.veml.config_idx);
            } else {
                msg.status = SENSOR_STATUS_ERROR;
                msg.data.veml.lux = 0.0f;
                ESP_LOGW(TAG, "VEML7700 read failed");
            }

            /* Send to queue */
            if (xQueueSend(sensor_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Sensor queue full, dropping VEML7700 reading");
            } else {
                ESP_LOGD(TAG, "Queued sensor data");
            }
        }

        /* Periodic delay (1 Hz for VEML7700) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/* ======================== CAN Tasks ======================== */

static void twai_receive_task(void *arg) {
    twai_message_t rx_msg;

    ESP_LOGI(TAG, "TWAI receive task started");

    while (1) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            if (rx_msg.identifier == ID_MASTER_STOP_CMD) {
                ESP_LOGI(TAG, "Received STOP command");
                xSemaphoreTake(done_sem, 0);  /* Clear if set */
                xSemaphoreGive(done_sem);      /* Signal stop */
            } else if (rx_msg.identifier == ID_MASTER_START_CMD) {
                ESP_LOGI(TAG, "Received START command");
                xSemaphoreTake(start_sem, 0);  /* Clear if set */
                xSemaphoreGive(start_sem);      /* Signal start */
            }
        }
    }
}

static void twai_transmit_task(void *arg) {
    twai_message_t tx_msg;
    sensor_data_t latest[NUM_SENSORS] = {0};
    sensor_data_t incoming;
    uint8_t sequence_counter = 0;
    TickType_t xLastWakeTime;

    ESP_LOGI(TAG, "TWAI transmit task started");

    while (1) {
        /* Wait for start command (blocking) */
        xSemaphoreTake(start_sem, portMAX_DELAY);

        sequence_counter = 0;
        xLastWakeTime = xTaskGetTickCount();

        ESP_LOGI(TAG, "Starting CAN transmission");

        while (1) {
            /* Check for stop command (non-blocking) */
            if (xSemaphoreTake(done_sem, 0) == pdTRUE) {
                ESP_LOGI(TAG, "Stopping CAN transmission");
                break;  /* Exit inner loop, wait for next START */
            }

            /* Collect latest readings from queue (non-blocking) */
            int queue_count = 0;
            while (xQueueReceive(sensor_queue, &incoming, 0) == pdTRUE) {
                latest[incoming.sensor_id] = incoming;
                queue_count++;
                ESP_LOGD(TAG, "Dequeued sensor %d: lux=%.1f",
                         incoming.sensor_id, incoming.data.veml.lux);
            }

            /* Transmit VEML7700 data */
            can_format_veml7700_message(&latest[SENSOR_VEML7700],
                                        sequence_counter,
                                        &tx_msg);

            ESP_LOGD(TAG, "TX: lux=%.1f, seq=%d, queue_items=%d",
                     latest[SENSOR_VEML7700].data.veml.lux, sequence_counter, queue_count);

            if (twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to transmit VEML7700 message");
            }

            sequence_counter++;

            /* Periodic delay (1 Hz) */
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_DATA_PERIOD_MS));
        }
    }
}

static void twai_control_task(void *arg) {
    ESP_LOGI(TAG, "TWAI control task started");

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Give start semaphore to begin transmission automatically */
    xSemaphoreGive(start_sem);
    ESP_LOGI(TAG, "Auto-start: Enabled sensor transmission");

    /* Task done - delete itself */
    vTaskDelete(NULL);
}

/* ======================== Main Application ======================== */

void app_main(void) {
    ESP_LOGI(TAG, "ESP32 Multi-Sensor CAN Node starting...");

    /* Create semaphores */
    done_sem = xSemaphoreCreateBinary();
    start_sem = xSemaphoreCreateBinary();

    if (done_sem == NULL || start_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return;
    }

    /* Create sensor queue */
    sensor_queue = xQueueCreate(SENSOR_QUEUE_DEPTH, sizeof(sensor_data_t));
    if (sensor_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor queue");
        return;
    }
    ESP_LOGI(TAG, "Sensor queue created (depth: %d)", SENSOR_QUEUE_DEPTH);

    /* Initialize VEML7700 sensor */
    if (veml7700_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize VEML7700");
        return;
    }
    ESP_LOGI(TAG, "VEML7700 initialized successfully");

    /* Configure and start TWAI (CAN) */
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

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver");
        return;
    }

    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI");
        return;
    }
    ESP_LOGI(TAG, "TWAI initialized at 500 kbps");

    /* Create tasks */
    xTaskCreate(sensor_poll_task, "SENSOR_POLL", 2048, NULL, SENSOR_POLL_TASK_PRIO, NULL);
    xTaskCreate(twai_receive_task, "TWAI_RX", 3072, NULL, CAN_RX_TASK_PRIO, NULL);
    xTaskCreate(twai_transmit_task, "TWAI_TX", 5120, NULL, CAN_TX_TASK_PRIO, NULL);
    xTaskCreate(twai_control_task, "TWAI_CTRL", 2048, NULL, CAN_CTRL_TSK_PRIO, NULL);

    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System ready - waiting for START command");

    /* Main task: Periodic status logging */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  /* 10 second interval */

        /* Read current lux for status */
        float current_lux;
        if (veml7700_read_lux(&current_lux) == ESP_OK) {
            ESP_LOGI(TAG, "Status: %.1f lux, Config: %d, Queue: %d/%d, Heap: %d KB",
                     current_lux,
                     veml7700_get_config_idx(),
                     uxQueueMessagesWaiting(sensor_queue),
                     SENSOR_QUEUE_DEPTH,
                     esp_get_free_heap_size() / 1024);
        }
    }
}
