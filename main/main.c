/* main.c - ESP32-C3/C6 Multi-Sensor CAN Node (Refactored) */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/twai.h>

#include "sensor_common.h"
#include "als_driver.h"
#include "can_protocol.h"

/* Conditional BME680 support */
#ifdef CONFIG_BME680_ENABLED
#include "bme680_bsec.h"
#endif

/* ======================== Configuration ======================== */

#define CAN_TX_GPIO_NUM         GPIO_NUM_4
#define CAN_RX_GPIO_NUM         GPIO_NUM_5

#define CAN_TX_TASK_PRIO        8
#define CAN_RX_TASK_PRIO        9
#define CAN_CTRL_TSK_PRIO       10
#define SENSOR_POLL_TASK_PRIO   5
#define BME680_TASK_PRIO        6  /* Between ALS and transmit */

#define CAN_DATA_PERIOD_MS      1000
#define SENSOR_QUEUE_DEPTH      20

/* BME680 sample rate from Kconfig (default 3 seconds for LP mode) */
#ifndef CONFIG_BME680_SAMPLE_RATE
#define BME680_SAMPLE_RATE_SEC  3  /* BSEC LP mode: 3 seconds */
#else
#define BME680_SAMPLE_RATE_SEC  CONFIG_BME680_SAMPLE_RATE
#endif

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
            /* Read ambient light sensor (VEML7700 or OPT4001) */
            msg.sensor_id = SENSOR_VEML7700;  // Keep same ID for CAN compatibility
            msg.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if (als_read_lux(&msg.data.veml.lux) == ESP_OK) {
                msg.status = SENSOR_STATUS_OK;
                msg.data.veml.config_idx = als_get_config_idx();
                ESP_LOGD(TAG, "Read ALS (%s): %.1f lux, config=%d",
                         als_get_sensor_name(), msg.data.veml.lux, msg.data.veml.config_idx);
            } else {
                msg.status = SENSOR_STATUS_ERROR;
                msg.data.veml.lux = 0.0f;
                ESP_LOGW(TAG, "ALS read failed");
            }

            /* Send to queue */
            if (xQueueSend(sensor_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Sensor queue full, dropping ALS reading");
            } else {
                ESP_LOGD(TAG, "Queued sensor data");
            }
        }

        /* Periodic delay (1 Hz for VEML7700) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/* ======================== BME680 Sensor Task ======================== */

#ifdef CONFIG_BME680_ENABLED
static void bme680_sensor_task(void *arg) {
    sensor_data_t msg;
    bme68x_data_t bme_data;

    ESP_LOGI(TAG, "BME680 sensor task started (fixed 3-second interval)");

    while (1) {
        /* Read BME680 sensor at fixed 3-second intervals */
        msg.sensor_id = SENSOR_BME680;
        msg.timestamp_ms = esp_timer_get_time() / 1000;

        esp_err_t read_result = bme680_read(&bme_data);
        if (read_result == ESP_OK) {
            msg.status = SENSOR_STATUS_OK;

            /* Copy data from BME68x structure to sensor_data_t */
            msg.data.bme.temperature = bme_data.temperature;
            msg.data.bme.humidity = bme_data.humidity;
            msg.data.bme.pressure = bme_data.pressure;
            msg.data.bme.iaq = bme_data.iaq;
            msg.data.bme.accuracy = bme_data.accuracy;
            msg.data.bme.co2_equiv = bme_data.co2_equiv;
            msg.data.bme.breath_voc = bme_data.breath_voc;

            ESP_LOGI(TAG, "BME680: T=%.1f°C, H=%.1f%%, P=%.1fhPa",
                     bme_data.temperature, bme_data.humidity, bme_data.pressure);

            /* Send to queue */
            if (xQueueSend(sensor_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Sensor queue full, dropping BME680 reading");
            }

            /* Fixed 3-second interval */
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else {
            ESP_LOGW(TAG, "BME680 read failed");
            vTaskDelay(pdMS_TO_TICKS(1000));  /* Wait 1 sec on error, retry */
        }
    }
}
#endif /* CONFIG_BME680_ENABLED */

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

            /* Transmit VEML7700 data (always present) */
            can_format_veml7700_message(&latest[SENSOR_VEML7700],
                                        sequence_counter,
                                        &tx_msg);

            ESP_LOGD(TAG, "TX: lux=%.1f, seq=%d, queue_items=%d",
                     latest[SENSOR_VEML7700].data.veml.lux, sequence_counter, queue_count);

            if (twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to transmit VEML7700 message");
            }

#ifdef CONFIG_BME680_ENABLED
            /* Transmit BME680 environmental data (if available) */
            if (latest[SENSOR_BME680].timestamp_ms > 0) {
                can_format_bme_env_message(&latest[SENSOR_BME680], &tx_msg);

                if (twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)) != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to transmit BME680 environmental message");
                } else {
                    ESP_LOGD(TAG, "TX BME ENV: T=%.1f°C, H=%.1f%%, P=%.1fhPa",
                             latest[SENSOR_BME680].data.bme.temperature,
                             latest[SENSOR_BME680].data.bme.humidity,
                             latest[SENSOR_BME680].data.bme.pressure);
                }

                /* Transmit BME680 air quality data */
                can_format_bme_aiq_message(&latest[SENSOR_BME680], &tx_msg);

                if (twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)) != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to transmit BME680 air quality message");
                } else {
                    ESP_LOGD(TAG, "TX BME AIQ: IAQ=%d, CO2=%d ppm, VOC=%d ppm",
                             latest[SENSOR_BME680].data.bme.iaq,
                             latest[SENSOR_BME680].data.bme.co2_equiv,
                             latest[SENSOR_BME680].data.bme.breath_voc);
                }
            }
#endif /* CONFIG_BME680_ENABLED */

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

    /* Initialize ambient light sensor (auto-detect VEML7700 or OPT4001) */
    if (als_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ambient light sensor");
        return;
    }
    ESP_LOGI(TAG, "Ambient light sensor (%s) initialized successfully", als_get_sensor_name());

#ifdef CONFIG_BME680_ENABLED
    /* Initialize BME680/BME688 environmental sensor */
    if (bme680_init() == ESP_OK) {
        ESP_LOGI(TAG, "BME680/688 (%s) initialized successfully (BSEC %s)",
                 bme680_get_variant_name(), bme680_get_bsec_version());
        ESP_LOGI(TAG, "BSEC calibration state age: %lu hours", bme680_get_state_age() / 3600);
    } else {
        ESP_LOGW(TAG, "BME680/688 not detected or initialization failed");
        ESP_LOGW(TAG, "Continuing without BME680 support");
    }
#endif /* CONFIG_BME680_ENABLED */

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
    xTaskCreate(sensor_poll_task, "SENSOR_POLL", 3072, NULL, SENSOR_POLL_TASK_PRIO, NULL);

#ifdef CONFIG_BME680_ENABLED
    xTaskCreate(bme680_sensor_task, "BME680_POLL", 10240, NULL, BME680_TASK_PRIO, NULL);
    ESP_LOGI(TAG, "BME680 sensor task created (10KB stack for BSEC)");
#endif

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
        if (als_read_lux(&current_lux) == ESP_OK) {
            ESP_LOGI(TAG, "Status: %s, %.1f lux, Config: %d, Queue: %d/%d, Heap: %d KB",
                     als_get_sensor_name(),
                     current_lux,
                     als_get_config_idx(),
                     uxQueueMessagesWaiting(sensor_queue),
                     SENSOR_QUEUE_DEPTH,
                     esp_get_free_heap_size() / 1024);
        }
    }
}
