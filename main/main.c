/* main.c - ESP32-C3/C6 Multi-Sensor CAN Node
 *
 * Supports two firmware types:
 *   FACTORY - Minimal firmware with CAN + OTA only (silent on CAN bus)
 *   MAIN    - Full-featured firmware with all sensors
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_ota_ops.h>
#include <nvs_flash.h>
#include <driver/twai.h>

#include "firmware_config.h"
#include "can_protocol.h"

/* OTA firmware update support (both FACTORY and MAIN) */
#include "ota_handler.h"

/* Sensor includes (MAIN firmware only) */
#if IS_MAIN_FIRMWARE
#include "sensor_common.h"
#include "als_driver.h"

/* Conditional BME680 support */
#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
#include "bme680_bsec.h"
#endif
#endif /* IS_MAIN_FIRMWARE */

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

/* NVS Configuration */
#define NVS_NAMESPACE_NODE      "node_config"
#define NVS_KEY_NODE_ID         "node_id"

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

#if IS_MAIN_FIRMWARE
static QueueHandle_t sensor_queue;
static bool g_tx_active = false;  /* Track if transmission is active */
#endif

/* Node ID configuration (0-5, loaded from NVS) */
static uint8_t g_node_id = 0;

/* ======================== NVS Node ID Functions ======================== */

static esp_err_t load_node_id_from_nvs(uint8_t *node_id) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE_NODE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        /* Namespace doesn't exist - use default */
        *node_id = 0;
        return ESP_ERR_NVS_NOT_FOUND;
    }

    err = nvs_get_u8(handle, NVS_KEY_NODE_ID, node_id);
    nvs_close(handle);

    if (err != ESP_OK) {
        *node_id = 0;  /* Default to node 0 */
    }

    /* Validate range */
    if (*node_id > CAN_MAX_NODE_ID) {
        ESP_LOGW(TAG, "Invalid node ID %d in NVS, defaulting to 0", *node_id);
        *node_id = 0;
    }

    return err;
}

static esp_err_t save_node_id_to_nvs(uint8_t node_id) {
    if (node_id > CAN_MAX_NODE_ID) {
        ESP_LOGE(TAG, "Invalid node ID %d (max %d)", node_id, CAN_MAX_NODE_ID);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE_NODE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(handle, NVS_KEY_NODE_ID, node_id);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Node ID %d saved to NVS", node_id);
    } else {
        ESP_LOGE(TAG, "Failed to save node ID: %s", esp_err_to_name(err));
    }

    return err;
}

/* ======================== Sensor Poll Task (MAIN only) ======================== */

#if IS_MAIN_FIRMWARE
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

#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
static void bme680_sensor_task(void *arg) {
    sensor_data_t msg;
    bme68x_data_t bme_data;

    ESP_LOGI(TAG, "BME680 sensor task started (BSEC-driven timing)");

    /* Wait for initial BSEC next_call time (set during init) */
    uint64_t next_call_ms = bme680_get_next_call_ms();
    if (next_call_ms > 0) {
        uint64_t now_ms = esp_timer_get_time() / 1000;
        if (next_call_ms > now_ms) {
            uint64_t wait_ms = next_call_ms - now_ms;
            ESP_LOGI(TAG, "Waiting %llu ms until first BSEC measurement", wait_ms);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
    }

    while (1) {
        /* Read BME680 sensor at BSEC-requested intervals */
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

            ESP_LOGI(TAG, "BME680: T=%.1f°C, H=%.1f%%, P=%.1fhPa, IAQ=%d (acc=%d), CO2=%dppm",
                     bme_data.temperature, bme_data.humidity, bme_data.pressure,
                     bme_data.iaq, bme_data.accuracy, bme_data.co2_equiv);

            /* Send to queue */
            if (xQueueSend(sensor_queue, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Sensor queue full, dropping BME680 reading");
            }

            /* Get next measurement time from BSEC */
            next_call_ms = bme680_get_next_call_ms();
            uint64_t now_ms = esp_timer_get_time() / 1000;
            if (next_call_ms > now_ms) {
                uint64_t wait_ms = next_call_ms - now_ms;
                ESP_LOGD(TAG, "Next BSEC call in %llu ms", wait_ms);
                vTaskDelay(pdMS_TO_TICKS(wait_ms));
            } else {
                /* Should not happen, but prevent tight loop */
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } else if (read_result == ESP_ERR_INVALID_STATE) {
            /* Too early - wait a bit and retry */
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            ESP_LOGW(TAG, "BME680 read failed");
            vTaskDelay(pdMS_TO_TICKS(1000));  /* Wait 1 sec on error */
        }
    }
}
#endif /* CONFIG_BME680_ENABLED */
#endif /* IS_MAIN_FIRMWARE */

/* ======================== CAN Tasks ======================== */

/* Helper function to get sensor flags */
static uint8_t get_sensor_flags(void) {
#if IS_MAIN_FIRMWARE
    uint8_t flags = 0;

    /* Check ALS sensor */
    if (als_get_sensor_name() != NULL) {
        flags |= SENSOR_FLAG_ALS;
    }

#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
    flags |= SENSOR_FLAG_BME680;
    if (bme680_is_bme688()) {
        flags |= SENSOR_FLAG_BME688;
    }
#endif

    return flags;
#else
    /* FACTORY firmware: No sensors */
    return 0;
#endif
}

/* Helper function to get ALS type for CAN INFO_RESPONSE */
static uint8_t get_als_type(void) {
#if IS_MAIN_FIRMWARE
    const char *name = als_get_sensor_name();
    if (name == NULL) return CAN_ALS_TYPE_NONE;

    /* Check sensor name to determine type */
    if (strstr(name, "VEML7700") != NULL) return CAN_ALS_TYPE_VEML7700;
    if (strstr(name, "OPT4001") != NULL) return CAN_ALS_TYPE_OPT4001;
    if (strstr(name, "OPT3001") != NULL) return CAN_ALS_TYPE_OPT3001;
#endif
    return CAN_ALS_TYPE_NONE;
}

/* Helper function to get partition info byte */
static uint8_t get_partition_info(void) {
    uint8_t partition_type = PARTITION_TYPE_UNKNOWN;
    uint8_t ota_state = OTA_IMG_STATE_UNDEFINED;

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running) {
        /* Determine partition type from subtype */
        switch (running->subtype) {
            case ESP_PARTITION_SUBTYPE_APP_FACTORY:
                partition_type = PARTITION_TYPE_FACTORY;
                break;
            case ESP_PARTITION_SUBTYPE_APP_OTA_0:
                partition_type = PARTITION_TYPE_OTA_0;
                break;
            case ESP_PARTITION_SUBTYPE_APP_OTA_1:
                partition_type = PARTITION_TYPE_OTA_1;
                break;
            default:
                partition_type = PARTITION_TYPE_UNKNOWN;
                break;
        }

        /* Get OTA image state (only meaningful for OTA partitions) */
        esp_ota_img_states_t img_state;
        if (esp_ota_get_state_partition(running, &img_state) == ESP_OK) {
            switch (img_state) {
                case ESP_OTA_IMG_NEW:
                    ota_state = OTA_IMG_STATE_NEW;
                    break;
                case ESP_OTA_IMG_PENDING_VERIFY:
                    ota_state = OTA_IMG_STATE_PENDING;
                    break;
                case ESP_OTA_IMG_VALID:
                    ota_state = OTA_IMG_STATE_VALID;
                    break;
                case ESP_OTA_IMG_INVALID:
                    ota_state = OTA_IMG_STATE_INVALID;
                    break;
                case ESP_OTA_IMG_ABORTED:
                    ota_state = OTA_IMG_STATE_ABORTED;
                    break;
                default:
                    ota_state = OTA_IMG_STATE_UNDEFINED;
                    break;
            }
        }
    }

    return PARTITION_INFO_PACK(partition_type, ota_state);
}

static void twai_receive_task(void *arg) {
    twai_message_t rx_msg;
    twai_message_t tx_msg;

    /* Calculate our node's message IDs */
    uint32_t my_stop_id     = CAN_MSG_ID(g_node_id, CAN_MSG_OFFSET_STOP);
    uint32_t my_start_id    = CAN_MSG_ID(g_node_id, CAN_MSG_OFFSET_START);
    uint32_t my_shutdown_id = CAN_MSG_ID(g_node_id, CAN_MSG_OFFSET_SHUTDOWN);
    uint32_t my_reboot_id   = CAN_MSG_ID(g_node_id, CAN_MSG_OFFSET_REBOOT);
    uint32_t my_factory_id  = CAN_MSG_ID(g_node_id, CAN_MSG_OFFSET_FACTORY_RST);
    uint32_t my_setid_id    = CAN_MSG_ID(g_node_id, CAN_MSG_OFFSET_SET_NODE_ID);
    uint32_t my_getinfo_id  = CAN_MSG_ID(g_node_id, CAN_MSG_OFFSET_GET_INFO);
    uint32_t my_ping_id     = CAN_MSG_ID(g_node_id, CAN_MSG_OFFSET_PING);

    ESP_LOGI(TAG, "TWAI receive task started (node %d, base 0x%03lX)",
             g_node_id, (unsigned long)CAN_BASE_ADDR(g_node_id));

    while (1) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            uint32_t id = rx_msg.identifier;

#if IS_MAIN_FIRMWARE
            /* MAIN firmware: Handle START/STOP/SHUTDOWN commands */
            if (id == my_stop_id) {
                ESP_LOGI(TAG, "Received STOP command");
                g_tx_active = false;
                xSemaphoreTake(done_sem, 0);  /* Clear if set */
                xSemaphoreGive(done_sem);      /* Signal stop */

            } else if (id == my_start_id) {
                ESP_LOGI(TAG, "Received START command");
                xSemaphoreTake(start_sem, 0);  /* Clear if set */
                xSemaphoreGive(start_sem);      /* Signal start */

            } else if (id == my_shutdown_id) {
                /* Graceful shutdown: Save state, stop transmission, keep running */
                ESP_LOGI(TAG, "Received SHUTDOWN command - saving state...");
#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
                if (bme680_save_state() == ESP_OK) {
                    ESP_LOGI(TAG, "BSEC calibration state saved successfully");
                } else {
                    ESP_LOGW(TAG, "Failed to save BSEC state");
                }
#endif
                ESP_LOGI(TAG, "Ready for power-off (ESP32 still running)");
                g_tx_active = false;
                xSemaphoreTake(done_sem, 0);
                xSemaphoreGive(done_sem);  /* Stop transmission */

            } else
#else
            /* FACTORY firmware: Ignore START/STOP/SHUTDOWN (no sensors) */
            if (id == my_stop_id || id == my_start_id || id == my_shutdown_id) {
                ESP_LOGD(TAG, "FACTORY mode: Ignoring command 0x%03lX (no sensors)",
                         (unsigned long)id);

            } else
#endif
            if (id == my_reboot_id) {
                /* Save state and reboot */
                ESP_LOGI(TAG, "Received REBOOT command - saving state and rebooting...");
#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
                bme680_save_state();  /* Save current calibration */
#endif
                vTaskDelay(pdMS_TO_TICKS(500));  /* Allow logs to flush */
                esp_restart();  /* Reboot ESP32 */

            } else if (id == my_factory_id) {
                /* Factory reset: Clear calibration and node ID, then reboot */
                ESP_LOGI(TAG, "Received FACTORY RESET command - clearing all config...");
#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
                if (bme680_reset_calibration() == ESP_OK) {
                    ESP_LOGI(TAG, "BSEC calibration cleared");
                } else {
                    ESP_LOGW(TAG, "Failed to clear BSEC calibration");
                }
#endif
                /* Clear node ID (reset to 0) */
                nvs_handle_t handle;
                if (nvs_open(NVS_NAMESPACE_NODE, NVS_READWRITE, &handle) == ESP_OK) {
                    nvs_erase_all(handle);
                    nvs_commit(handle);
                    nvs_close(handle);
                    ESP_LOGI(TAG, "Node config cleared (will use default node ID 0)");
                }
                ESP_LOGI(TAG, "Rebooting to apply factory reset...");
                vTaskDelay(pdMS_TO_TICKS(500));  /* Allow logs to flush */
                esp_restart();  /* Reboot ESP32 */

            } else if (id == my_setid_id) {
                /* Set new node ID */
                if (rx_msg.data_length_code >= 1) {
                    uint8_t new_id = rx_msg.data[0];
                    ESP_LOGI(TAG, "Received SET_NODE_ID command: %d -> %d", g_node_id, new_id);

                    if (new_id > CAN_MAX_NODE_ID) {
                        ESP_LOGW(TAG, "Invalid node ID %d (max %d), ignoring", new_id, CAN_MAX_NODE_ID);
                    } else if (new_id == g_node_id) {
                        ESP_LOGI(TAG, "Node ID unchanged, no reboot needed");
                    } else {
                        /* Save new node ID and reboot */
                        if (save_node_id_to_nvs(new_id) == ESP_OK) {
#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
                            bme680_save_state();  /* Save calibration before reboot */
#endif
                            ESP_LOGI(TAG, "Rebooting with new node ID %d...", new_id);
                            vTaskDelay(pdMS_TO_TICKS(500));
                            esp_restart();
                        } else {
                            ESP_LOGE(TAG, "Failed to save node ID, not rebooting");
                        }
                    }
                }

            } else if (id == my_getinfo_id) {
                /* Respond with device info */
                ESP_LOGI(TAG, "Received GET_INFO command");

#if IS_MAIN_FIRMWARE
                uint8_t status_flags = g_tx_active ? STATUS_FLAG_TX_ACTIVE : 0;
#else
                uint8_t status_flags = 0;  /* FACTORY: TX never active (no sensors) */
#endif
                uint8_t partition_info = get_partition_info();
                can_format_info_response(g_node_id, get_sensor_flags(),
                                         get_als_type(), status_flags,
                                         partition_info, &tx_msg);

                if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
                    ESP_LOGI(TAG, "Sent INFO_RESPONSE (node=%d, v%d.%d.%d, part=%d)",
                             g_node_id, FIRMWARE_VERSION_MAJOR,
                             FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH,
                             PARTITION_INFO_TYPE(partition_info));
                } else {
                    ESP_LOGW(TAG, "Failed to send INFO_RESPONSE");
                }

            } else if (id == my_ping_id) {
                /* Respond with PONG for discovery */
                ESP_LOGD(TAG, "Received PING, sending PONG");
                can_format_pong_response(g_node_id, &tx_msg);

                if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to send PONG");
                }

            } else {
                /* Check if this is an OTA message */
                esp_err_t ota_result = ota_handler_process_message(&rx_msg);
                if (ota_result == ESP_OK) {
                    /* OTA message was handled */
                    ESP_LOGD(TAG, "Processed OTA message");
                }
                /* If not OTA message (ESP_ERR_INVALID_ARG), ignore silently */
            }
        }
    }
}

#if IS_MAIN_FIRMWARE
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
        g_tx_active = true;

        ESP_LOGI(TAG, "Starting CAN transmission (node %d)", g_node_id);

        while (1) {
            /* Check for stop command (non-blocking) */
            if (xSemaphoreTake(done_sem, 0) == pdTRUE) {
                ESP_LOGI(TAG, "Stopping CAN transmission");
                g_tx_active = false;
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

            /* Transmit VEML7700/OPT4001 data (always present) */
            can_format_veml7700_message(&latest[SENSOR_VEML7700],
                                        sequence_counter,
                                        &tx_msg);
            /* Apply node ID offset to message ID */
            can_set_msg_id(&tx_msg, g_node_id, CAN_MSG_OFFSET_ALS);

            ESP_LOGD(TAG, "TX: lux=%.1f, seq=%d, queue_items=%d",
                     latest[SENSOR_VEML7700].data.veml.lux, sequence_counter, queue_count);

            if (twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to transmit ALS message");
            }

#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
            /* Transmit BME680 environmental data (if available) */
            if (latest[SENSOR_BME680].timestamp_ms > 0) {
                can_format_bme_env_message(&latest[SENSOR_BME680], &tx_msg);
                can_set_msg_id(&tx_msg, g_node_id, CAN_MSG_OFFSET_ENV);

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
                can_set_msg_id(&tx_msg, g_node_id, CAN_MSG_OFFSET_AIQ);

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
#endif /* IS_MAIN_FIRMWARE */

static void twai_control_task(void *arg) {
    ESP_LOGI(TAG, "TWAI control task started");

    vTaskDelay(pdMS_TO_TICKS(100));

#if AUTO_START_TX
    /* MAIN firmware: Auto-start sensor transmission */
    xSemaphoreGive(start_sem);
    ESP_LOGI(TAG, "Auto-start: Enabled sensor transmission");
#else
    /* FACTORY firmware: Stay silent, wait for commands */
    ESP_LOGI(TAG, "FACTORY mode: Ready for OTA updates (silent on CAN bus)");
#endif

    /* Task done - delete itself */
    vTaskDelete(NULL);
}

/* ======================== Main Application ======================== */

void app_main(void) {
    ESP_LOGI(TAG, "ESP32 CAN Node v%d.%d.%d (%s) starting...",
             FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH,
             FIRMWARE_TYPE_STR);

    /* Initialize NVS flash (required for calibration and node ID storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS flash initialized");

    /* Load node ID from NVS */
    if (load_node_id_from_nvs(&g_node_id) == ESP_OK) {
        ESP_LOGI(TAG, "Loaded node ID %d from NVS (base address 0x%03lX)",
                 g_node_id, (unsigned long)CAN_BASE_ADDR(g_node_id));
    } else {
        ESP_LOGI(TAG, "Using default node ID %d (base address 0x%03lX)",
                 g_node_id, (unsigned long)CAN_BASE_ADDR(g_node_id));
    }

    /* Initialize OTA handler */
    if (ota_handler_init(g_node_id) == ESP_OK) {
        ESP_LOGI(TAG, "OTA handler initialized (CAN ID 0x%03lX)",
                 (unsigned long)CAN_OTA_CMD_ID(g_node_id));
        /* Mark firmware as valid to prevent rollback */
        ota_handler_mark_valid();
    } else {
        ESP_LOGW(TAG, "OTA handler initialization failed");
    }

    /* Create semaphores */
    done_sem = xSemaphoreCreateBinary();
    start_sem = xSemaphoreCreateBinary();

    if (done_sem == NULL || start_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return;
    }

#if IS_MAIN_FIRMWARE
    /* Create sensor queue (MAIN firmware only) */
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

#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
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
#endif /* IS_MAIN_FIRMWARE */

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
    /* Create CAN receive task (both FACTORY and MAIN) */
    xTaskCreate(twai_receive_task, "TWAI_RX", 3072, NULL, CAN_RX_TASK_PRIO, NULL);

#if IS_MAIN_FIRMWARE
    /* Create sensor tasks (MAIN firmware only) */
    xTaskCreate(sensor_poll_task, "SENSOR_POLL", 3072, NULL, SENSOR_POLL_TASK_PRIO, NULL);

#if defined(CONFIG_BME680_ENABLED) && BSEC_LIBRARY_AVAILABLE
    xTaskCreate(bme680_sensor_task, "BME680_POLL", 10240, NULL, BME680_TASK_PRIO, NULL);
    ESP_LOGI(TAG, "BME680 sensor task created (10KB stack for BSEC)");
#endif

    /* Create CAN transmit task (MAIN firmware only) */
    xTaskCreate(twai_transmit_task, "TWAI_TX", 5120, NULL, CAN_TX_TASK_PRIO, NULL);
#endif /* IS_MAIN_FIRMWARE */

    /* Create control task (both, but behavior differs) */
    xTaskCreate(twai_control_task, "TWAI_CTRL", 2048, NULL, CAN_CTRL_TSK_PRIO, NULL);

    ESP_LOGI(TAG, "All tasks created successfully");
#if IS_MAIN_FIRMWARE
    ESP_LOGI(TAG, "System ready - sensor transmission will auto-start");
#else
    ESP_LOGI(TAG, "FACTORY mode ready - awaiting OTA updates");
#endif

    /* Main task: Periodic status logging and OTA timeout check */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  /* 10 second interval */

        /* Check for OTA session timeout */
        ota_handler_check_timeout();

#if IS_MAIN_FIRMWARE
        /* Read current lux for status (MAIN firmware only) */
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
#else
        /* FACTORY firmware: Just log heap status */
        ESP_LOGI(TAG, "FACTORY status: Heap: %d KB free", esp_get_free_heap_size() / 1024);
#endif
    }
}
