/* ota_handler.h - OTA Firmware Update Handler over CAN */

#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/twai.h"
#include "can_protocol.h"

/* ======================== OTA Configuration ======================== */

#define OTA_CHUNK_SIZE          6       /* Bytes per CAN frame payload */
#define OTA_ACK_TIMEOUT_MS      500     /* Timeout for ACK response */
#define OTA_SESSION_TIMEOUT_MS  300000  /* 5 minutes session timeout */
#define OTA_MAX_RETRIES         3       /* Max retries per chunk */

/* ======================== OTA Context Structure ======================== */

typedef struct {
    ota_state_t state;              /* Current state machine state */

    /* Update metadata */
    uint32_t firmware_size;         /* Total firmware size in bytes */
    uint32_t expected_crc32;        /* Expected CRC32 of complete firmware */
    uint16_t expected_crc16;        /* Truncated CRC for START_UPDATE */

    /* Progress tracking */
    uint32_t bytes_received;        /* Total bytes received so far */
    uint8_t expected_seq;           /* Expected next sequence number */
    uint8_t last_error;             /* Last error code */

    /* ESP-IDF OTA handle */
    void *ota_handle;               /* esp_ota_handle_t */
    void *update_partition;         /* esp_partition_t* */

    /* Timing */
    uint32_t last_activity_ms;      /* Last activity timestamp */

    /* Node ID for response addressing */
    uint8_t node_id;
} ota_context_t;

/* ======================== OTA Handler Functions ======================== */

/**
 * @brief Initialize OTA handler
 *
 * Sets up the OTA context and prepares for firmware updates.
 * Must be called once at startup.
 *
 * @param[in] node_id Current node ID for CAN addressing
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_handler_init(uint8_t node_id);

/**
 * @brief Process incoming OTA CAN message
 *
 * Handles OTA commands received via CAN bus. Should be called
 * from the CAN receive task when an OTA message is received.
 *
 * @param[in] msg Received CAN message
 * @return ESP_OK if message was handled, ESP_ERR_INVALID_ARG if not an OTA message
 */
esp_err_t ota_handler_process_message(const twai_message_t *msg);

/**
 * @brief Check for OTA session timeout
 *
 * Should be called periodically to check for stale OTA sessions.
 * If a session has been idle for too long, it will be aborted.
 */
void ota_handler_check_timeout(void);

/**
 * @brief Get current OTA state
 *
 * @return Current OTA state machine state
 */
ota_state_t ota_handler_get_state(void);

/**
 * @brief Get OTA progress percentage
 *
 * @return Progress as 0-100 percentage, or 0 if no update in progress
 */
uint8_t ota_handler_get_progress(void);

/**
 * @brief Check if OTA update is in progress
 *
 * @return true if update is in progress, false otherwise
 */
bool ota_handler_is_busy(void);

/**
 * @brief Abort current OTA update
 *
 * Cancels any in-progress update and cleans up resources.
 *
 * @return ESP_OK on success
 */
esp_err_t ota_handler_abort(void);

/**
 * @brief Mark current firmware as valid (anti-rollback)
 *
 * Should be called after successful boot to prevent rollback
 * to the previous firmware version.
 *
 * @return ESP_OK on success
 */
esp_err_t ota_handler_mark_valid(void);

/**
 * @brief Get running partition info
 *
 * @param[out] label Partition label (e.g., "factory", "ota_0", "ota_1")
 * @param[out] address Partition start address
 * @param[out] size Partition size
 * @return ESP_OK on success
 */
esp_err_t ota_handler_get_running_partition(const char **label,
                                             uint32_t *address,
                                             uint32_t *size);

#endif /* OTA_HANDLER_H */
