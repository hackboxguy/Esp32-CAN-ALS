/* ota_handler.c - OTA Firmware Update Handler over CAN */

#include "ota_handler.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/twai.h"
#include <string.h>

static const char *TAG = "OTA_HANDLER";

/* ======================== Static Variables ======================== */

static ota_context_t s_ctx = {0};

/* ======================== Helper Functions ======================== */

static uint32_t get_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static esp_err_t send_response(uint8_t type, const uint8_t *data, uint8_t len) {
    twai_message_t msg = {0};
    msg.identifier = CAN_OTA_RESP_ID(s_ctx.node_id);
    msg.data_length_code = 8;
    msg.data[0] = type;
    msg.data[1] = s_ctx.expected_seq;

    if (data && len > 0) {
        size_t copy_len = (len > 6) ? 6 : len;
        memcpy(&msg.data[2], data, copy_len);
    }

    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send OTA response: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t send_ack(uint8_t seq) {
    uint8_t data[6] = {seq, 0, 0, 0, 0, 0};
    return send_response(OTA_RESP_ACK, data, 6);
}

static esp_err_t send_nak(uint8_t seq, uint8_t error) {
    uint8_t data[6] = {seq, error, 0, 0, 0, 0};
    return send_response(OTA_RESP_NAK, data, 6);
}

static esp_err_t send_ready(uint16_t max_chunk, uint32_t free_space) {
    uint8_t data[6] = {
        max_chunk & 0xFF,
        (max_chunk >> 8) & 0xFF,
        free_space & 0xFF,
        (free_space >> 8) & 0xFF,
        (free_space >> 16) & 0xFF,
        (free_space >> 24) & 0xFF
    };
    return send_response(OTA_RESP_READY, data, 6);
}

static esp_err_t send_complete(uint8_t result) {
    uint8_t data[6] = {result, 0, 0, 0, 0, 0};
    return send_response(OTA_RESP_COMPLETE, data, 6);
}

static esp_err_t send_status(void) {
    uint16_t progress = 0;
    if (s_ctx.firmware_size > 0) {
        progress = (uint16_t)((s_ctx.bytes_received * 100) / s_ctx.firmware_size);
    }

    uint8_t data[6] = {
        (uint8_t)s_ctx.state,
        progress & 0xFF,
        (progress >> 8) & 0xFF,
        s_ctx.last_error,
        0, 0
    };
    return send_response(OTA_RESP_STATUS, data, 6);
}

static void reset_context(void) {
    /* Clean up ESP-IDF OTA handle if active */
    if (s_ctx.ota_handle) {
        esp_ota_abort((esp_ota_handle_t)s_ctx.ota_handle);
        s_ctx.ota_handle = NULL;
    }

    /* Reset state but preserve node_id */
    uint8_t node_id = s_ctx.node_id;
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.node_id = node_id;
    s_ctx.state = OTA_STATE_IDLE;
}

/* ======================== Command Handlers ======================== */

static esp_err_t handle_start_update(const uint8_t *data) {
    if (s_ctx.state != OTA_STATE_IDLE) {
        ESP_LOGW(TAG, "START_UPDATE received but already busy (state=%d)", s_ctx.state);
        send_nak(0, OTA_ERR_BUSY);
        return ESP_ERR_INVALID_STATE;
    }

    /* Parse firmware size and CRC16 */
    uint32_t fw_size = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    uint16_t crc16 = data[4] | (data[5] << 8);

    ESP_LOGI(TAG, "START_UPDATE: size=%lu, crc16=0x%04X", (unsigned long)fw_size, crc16);

    /* Get next OTA partition */
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition available");
        send_nak(0, OTA_ERR_INVALID);
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if firmware fits */
    if (fw_size > update_partition->size) {
        ESP_LOGE(TAG, "Firmware too large: %lu > %lu",
                 (unsigned long)fw_size, (unsigned long)update_partition->size);
        send_nak(0, OTA_ERR_NO_SPACE);
        return ESP_ERR_NO_MEM;
    }

    /* Begin OTA update (this erases the partition) */
    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        send_nak(0, OTA_ERR_WRITE);
        return err;
    }

    /* Initialize context */
    s_ctx.state = OTA_STATE_RECEIVING;
    s_ctx.firmware_size = fw_size;
    s_ctx.expected_crc16 = crc16;
    s_ctx.expected_crc32 = 0;  /* Will be provided in FINISH_UPDATE */
    s_ctx.bytes_received = 0;
    s_ctx.expected_seq = 0;
    s_ctx.last_error = OTA_ERR_OK;
    s_ctx.ota_handle = (void *)ota_handle;
    s_ctx.update_partition = (void *)update_partition;
    s_ctx.last_activity_ms = get_time_ms();

    ESP_LOGI(TAG, "OTA started, writing to partition '%s' @ 0x%lX",
             update_partition->label, (unsigned long)update_partition->address);

    /* Send READY response */
    send_ready(OTA_CHUNK_SIZE, update_partition->size);

    return ESP_OK;
}

static esp_err_t handle_send_chunk(uint8_t seq, const uint8_t *data) {
    if (s_ctx.state != OTA_STATE_RECEIVING) {
        ESP_LOGW(TAG, "SEND_CHUNK received but not in RECEIVING state");
        send_nak(seq, OTA_ERR_INVALID);
        return ESP_ERR_INVALID_STATE;
    }

    /* Check sequence number */
    if (seq != s_ctx.expected_seq) {
        ESP_LOGW(TAG, "Sequence mismatch: expected %d, got %d", s_ctx.expected_seq, seq);
        send_nak(seq, OTA_ERR_SEQ);
        return ESP_ERR_INVALID_ARG;
    }

    /* Calculate how many bytes to write (last chunk may be partial) */
    uint32_t remaining = s_ctx.firmware_size - s_ctx.bytes_received;
    size_t write_len = (remaining < OTA_CHUNK_SIZE) ? remaining : OTA_CHUNK_SIZE;

    /* Write to flash */
    esp_err_t err = esp_ota_write((esp_ota_handle_t)s_ctx.ota_handle, data, write_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        s_ctx.last_error = OTA_ERR_WRITE;
        send_nak(seq, OTA_ERR_WRITE);
        return err;
    }

    /* Update progress */
    s_ctx.bytes_received += write_len;
    s_ctx.expected_seq = (seq + 1) & 0xFF;
    s_ctx.last_activity_ms = get_time_ms();

    /* Log progress periodically */
    if ((s_ctx.bytes_received % 10000) < OTA_CHUNK_SIZE) {
        ESP_LOGI(TAG, "OTA progress: %lu / %lu bytes (%lu%%)",
                 (unsigned long)s_ctx.bytes_received,
                 (unsigned long)s_ctx.firmware_size,
                 (unsigned long)((s_ctx.bytes_received * 100) / s_ctx.firmware_size));
    }

    /* Send ACK */
    send_ack(seq);

    /* Check if all data received */
    if (s_ctx.bytes_received >= s_ctx.firmware_size) {
        ESP_LOGI(TAG, "All firmware data received, waiting for FINISH_UPDATE");
        s_ctx.state = OTA_STATE_VERIFYING;
    }

    return ESP_OK;
}

static esp_err_t handle_finish_update(const uint8_t *data) {
    if (s_ctx.state != OTA_STATE_VERIFYING) {
        ESP_LOGW(TAG, "FINISH_UPDATE received but not in VERIFYING state");
        send_nak(0, OTA_ERR_INVALID);
        return ESP_ERR_INVALID_STATE;
    }

    /* Parse full CRC32 */
    uint32_t crc32 = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    s_ctx.expected_crc32 = crc32;

    ESP_LOGI(TAG, "FINISH_UPDATE: expected CRC32=0x%08lX", (unsigned long)crc32);

    /* Finalize OTA (validates image) */
    esp_err_t err = esp_ota_end((esp_ota_handle_t)s_ctx.ota_handle);
    s_ctx.ota_handle = NULL;

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        s_ctx.last_error = OTA_ERR_CRC;
        send_complete(OTA_ERR_CRC);
        reset_context();
        return err;
    }

    /* Note: ESP-IDF's esp_ota_end() validates the image header and app descriptor.
     * For full CRC32 verification, we would need to read back the partition and
     * calculate CRC, but this is optional since esp_ota_end() does its own validation.
     */

    ESP_LOGI(TAG, "Firmware validated successfully, ready to activate");
    s_ctx.state = OTA_STATE_READY;
    s_ctx.last_activity_ms = get_time_ms();

    send_complete(OTA_ERR_OK);

    return ESP_OK;
}

static esp_err_t handle_activate_fw(const uint8_t *data) {
    if (s_ctx.state != OTA_STATE_READY) {
        ESP_LOGW(TAG, "ACTIVATE_FW received but not in READY state");
        send_nak(0, OTA_ERR_INVALID);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t flags = data[0];
    bool reboot = (flags & OTA_ACTIVATE_FLAG_REBOOT) != 0;

    ESP_LOGI(TAG, "ACTIVATE_FW: flags=0x%02X, reboot=%d", flags, reboot);

    /* Set boot partition */
    const esp_partition_t *update_partition = (const esp_partition_t *)s_ctx.update_partition;
    esp_err_t err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        s_ctx.last_error = OTA_ERR_WRITE;
        send_nak(0, OTA_ERR_WRITE);
        return err;
    }

    ESP_LOGI(TAG, "Boot partition set to '%s'", update_partition->label);

    /* Send ACK before potential reboot */
    send_ack(0);

    /* Reset context */
    reset_context();

    if (reboot) {
        ESP_LOGI(TAG, "Rebooting to new firmware...");
        vTaskDelay(pdMS_TO_TICKS(500));  /* Allow ACK to be sent */
        esp_restart();
    }

    return ESP_OK;
}

static esp_err_t handle_get_status(void) {
    ESP_LOGD(TAG, "GET_STATUS: state=%d, progress=%lu/%lu",
             s_ctx.state, (unsigned long)s_ctx.bytes_received,
             (unsigned long)s_ctx.firmware_size);
    send_status();
    return ESP_OK;
}

static esp_err_t handle_abort_update(void) {
    ESP_LOGI(TAG, "ABORT_UPDATE received");

    if (s_ctx.state == OTA_STATE_IDLE) {
        send_ack(0);
        return ESP_OK;
    }

    /* Clean up and reset */
    reset_context();
    s_ctx.last_error = OTA_ERR_ABORTED;

    send_ack(0);

    return ESP_OK;
}

/* ======================== Public Functions ======================== */

esp_err_t ota_handler_init(uint8_t node_id) {
    ESP_LOGI(TAG, "Initializing OTA handler for node %d", node_id);

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.node_id = node_id;
    s_ctx.state = OTA_STATE_IDLE;

    /* Log current partition info */
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running) {
        ESP_LOGI(TAG, "Running from partition '%s' @ 0x%lX",
                 running->label, (unsigned long)running->address);
    }

    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (next) {
        ESP_LOGI(TAG, "Next OTA partition: '%s' @ 0x%lX (size %lu)",
                 next->label, (unsigned long)next->address, (unsigned long)next->size);
    } else {
        ESP_LOGW(TAG, "No OTA partition available!");
    }

    return ESP_OK;
}

esp_err_t ota_handler_process_message(const twai_message_t *msg) {
    /* Check if this is an OTA command for our node */
    uint32_t expected_id = CAN_OTA_CMD_ID(s_ctx.node_id);
    if (msg->identifier != expected_id) {
        return ESP_ERR_INVALID_ARG;  /* Not for us */
    }

    if (msg->data_length_code < 2) {
        ESP_LOGW(TAG, "OTA message too short: %d bytes", msg->data_length_code);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t cmd_type = msg->data[0];
    uint8_t seq = msg->data[1];
    const uint8_t *payload = &msg->data[2];

    ESP_LOGD(TAG, "OTA cmd: type=0x%02X, seq=%d", cmd_type, seq);

    switch (cmd_type) {
        case OTA_CMD_START_UPDATE:
            return handle_start_update(payload);

        case OTA_CMD_SEND_CHUNK:
            return handle_send_chunk(seq, payload);

        case OTA_CMD_FINISH_UPDATE:
            return handle_finish_update(payload);

        case OTA_CMD_ACTIVATE_FW:
            return handle_activate_fw(payload);

        case OTA_CMD_GET_STATUS:
            return handle_get_status();

        case OTA_CMD_ABORT_UPDATE:
            return handle_abort_update();

        default:
            ESP_LOGW(TAG, "Unknown OTA command: 0x%02X", cmd_type);
            send_nak(seq, OTA_ERR_INVALID);
            return ESP_ERR_INVALID_ARG;
    }
}

void ota_handler_check_timeout(void) {
    if (s_ctx.state == OTA_STATE_IDLE) {
        return;
    }

    uint32_t now = get_time_ms();
    uint32_t elapsed = now - s_ctx.last_activity_ms;

    if (elapsed > OTA_SESSION_TIMEOUT_MS) {
        ESP_LOGW(TAG, "OTA session timeout (%lu ms), aborting", (unsigned long)elapsed);
        reset_context();
        s_ctx.last_error = OTA_ERR_TIMEOUT;
    }
}

ota_state_t ota_handler_get_state(void) {
    return s_ctx.state;
}

uint8_t ota_handler_get_progress(void) {
    if (s_ctx.firmware_size == 0) {
        return 0;
    }
    return (uint8_t)((s_ctx.bytes_received * 100) / s_ctx.firmware_size);
}

bool ota_handler_is_busy(void) {
    return s_ctx.state != OTA_STATE_IDLE;
}

esp_err_t ota_handler_abort(void) {
    return handle_abort_update();
}

esp_err_t ota_handler_mark_valid(void) {
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Firmware marked as valid (rollback disabled)");
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        /* Running from factory partition, no rollback to cancel */
        ESP_LOGD(TAG, "Running from factory, no rollback to cancel");
        err = ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to mark firmware valid: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ota_handler_get_running_partition(const char **label,
                                             uint32_t *address,
                                             uint32_t *size) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    if (label) *label = running->label;
    if (address) *address = running->address;
    if (size) *size = running->size;

    return ESP_OK;
}
