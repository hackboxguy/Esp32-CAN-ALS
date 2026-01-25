/* can_protocol.h - CAN Message Protocol Definitions and Formatting */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <driver/twai.h>
#include "sensor_common.h"

/* ======================== Firmware Version ======================== */

#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0

/* ======================== Node ID Addressing ======================== */

/* Each node has 16 message IDs reserved (0x10 spacing) */
#define CAN_NODE_ADDR_SPACING   0x10
#define CAN_BASE_ADDR_NODE_0    0x0A0
#define CAN_MAX_NODE_ID         5

/* Calculate base address for a given node ID */
#define CAN_BASE_ADDR(node_id)  (CAN_BASE_ADDR_NODE_0 + ((node_id) * CAN_NODE_ADDR_SPACING))

/* Message offsets from node base address */
#define CAN_MSG_OFFSET_STOP         0x00  /* Stop transmission */
#define CAN_MSG_OFFSET_START        0x01  /* Start transmission */
#define CAN_MSG_OFFSET_ALS          0x02  /* Ambient light data */
#define CAN_MSG_OFFSET_ENV          0x03  /* Environmental data (T/H/P) */
#define CAN_MSG_OFFSET_AIQ          0x04  /* Air quality (IAQ/CO2/VOC) */
#define CAN_MSG_OFFSET_LD2410       0x05  /* Presence detection */
#define CAN_MSG_OFFSET_MQ3          0x06  /* Alcohol/VOC */
#define CAN_MSG_OFFSET_STATUS       0x07  /* System status */
#define CAN_MSG_OFFSET_SHUTDOWN     0x08  /* Graceful shutdown */
#define CAN_MSG_OFFSET_REBOOT       0x09  /* Reboot */
#define CAN_MSG_OFFSET_FACTORY_RST  0x0A  /* Factory reset */
#define CAN_MSG_OFFSET_SET_NODE_ID  0x0B  /* Set node ID */
#define CAN_MSG_OFFSET_GET_INFO     0x0C  /* Request device info */
#define CAN_MSG_OFFSET_INFO_RESP    0x0D  /* Device info response */
#define CAN_MSG_OFFSET_PING         0x0E  /* Discovery ping */
#define CAN_MSG_OFFSET_PONG         0x0F  /* Discovery response */

/* Calculate CAN ID for a given node and message type */
#define CAN_MSG_ID(node_id, offset) (CAN_BASE_ADDR(node_id) + (offset))

/* ======================== CAN Message IDs (Node 0 defaults) ======================== */

/* Control Messages (RX - from master to ESP32) */
#define ID_MASTER_STOP_CMD      0x0A0  /* Stop sensor transmission */
#define ID_MASTER_START_CMD     0x0A1  /* Start sensor transmission */
#define ID_MASTER_SHUTDOWN_CMD  0x0A8  /* Graceful shutdown (save state, keep running) */
#define ID_MASTER_REBOOT_CMD    0x0A9  /* Save state and reboot ESP32 */
#define ID_MASTER_FACTORY_RST   0x0AA  /* Factory reset (clear calibration, reboot) */
#define ID_MASTER_SET_NODE_ID   0x0AB  /* Set node ID (triggers reboot) */
#define ID_MASTER_GET_INFO      0x0AC  /* Request device info */
#define ID_MASTER_PING          0x0AE  /* Discovery ping */

/* Sensor Data Messages (TX - from ESP32 to master) */
#define ID_SENSOR_VEML7700      0x0A2  /* Ambient light (1 Hz) */
#define ID_SENSOR_BME_ENV       0x0A3  /* BME680/688 environmental (0.33 Hz) */
#define ID_SENSOR_BME_AIQ       0x0A4  /* BME680/688 air quality (0.33 Hz) */
#define ID_SENSOR_LD2410        0x0A5  /* Presence detection (10 Hz) */
#define ID_SENSOR_MQ3           0x0A6  /* Alcohol/VOC (1 Hz) */
#define ID_SENSOR_STATUS        0x0A7  /* System status (0.1 Hz) */
#define ID_SENSOR_INFO_RESP     0x0AD  /* Device info response */
#define ID_SENSOR_PONG          0x0AF  /* Discovery response */

/* Legacy alias for backward compatibility */
#define ID_MASTER_DATA          ID_SENSOR_VEML7700

/* ======================== Sensor Flags ======================== */

/* Sensor presence flags for INFO_RESPONSE message */
#define SENSOR_FLAG_ALS         (1 << 0)  /* Ambient light sensor present */
#define SENSOR_FLAG_BME680      (1 << 1)  /* BME680/688 present */
#define SENSOR_FLAG_LD2410      (1 << 2)  /* LD2410 presence sensor */
#define SENSOR_FLAG_MQ3         (1 << 3)  /* MQ-3 gas sensor */

/* ALS type values for INFO_RESPONSE */
#define ALS_TYPE_NONE           0
#define ALS_TYPE_VEML7700       1
#define ALS_TYPE_OPT4001        2

/* Status flags for INFO_RESPONSE */
#define STATUS_FLAG_TX_ACTIVE   (1 << 0)  /* Currently transmitting */

/* ======================== Message Formatting Functions ======================== */

/**
 * @brief Format VEML7700 lux sensor data into CAN message
 *
 * Message format (8 bytes):
 *   Byte 0-2: Lux value (uint24_t, little-endian, 0-16,777,215)
 *   Byte 3:   Sensor status (0x00=OK, 0x01=Error)
 *   Byte 4:   Sequence counter (0-255, rolls over)
 *   Byte 5:   Current config index (0-20)
 *   Byte 6-7: Checksum (sum of bytes 0-5, uint16_t LE)
 *
 * @param[in] data Sensor data structure
 * @param[in] sequence Sequence counter value
 * @param[out] msg CAN message to populate
 */
void can_format_veml7700_message(const sensor_data_t *data, uint8_t sequence, twai_message_t *msg);

/**
 * @brief Format BME680/688 environmental data into CAN message
 *
 * Message format (8 bytes):
 *   Byte 0-1: Temperature (int16_t, °C × 100, e.g., 2350 = 23.50°C)
 *   Byte 2:   Humidity (uint8_t, %RH)
 *   Byte 3-4: Pressure (uint16_t, hPa × 10, e.g., 10132 = 1013.2 hPa)
 *   Byte 5:   Status
 *   Byte 6-7: Checksum
 *
 * @param[in] data Sensor data structure
 * @param[out] msg CAN message to populate
 */
void can_format_bme_env_message(const sensor_data_t *data, twai_message_t *msg);

/**
 * @brief Format BME680/688 air quality data into CAN message
 *
 * Message format (8 bytes):
 *   Byte 0-1: IAQ (uint16_t, 0-500)
 *   Byte 2:   Accuracy (uint8_t, 0-3)
 *   Byte 3-4: CO2 equivalent (uint16_t, ppm)
 *   Byte 5-6: Breath VOC (uint16_t, ppm)
 *   Byte 7:   Status
 *
 * @param[in] data Sensor data structure
 * @param[out] msg CAN message to populate
 */
void can_format_bme_aiq_message(const sensor_data_t *data, twai_message_t *msg);

/**
 * @brief Format LD2410 presence detection data into CAN message
 *
 * Message format (8 bytes):
 *   Byte 0:   Presence (1=detected, 0=none)
 *   Byte 1-2: Distance (uint16_t, cm)
 *   Byte 3:   Move energy (0-100)
 *   Byte 4:   Static energy (0-100)
 *   Byte 5:   Status
 *   Byte 6-7: Checksum
 *
 * @param[in] data Sensor data structure
 * @param[out] msg CAN message to populate
 */
void can_format_ld2410_message(const sensor_data_t *data, twai_message_t *msg);

/**
 * @brief Format MQ-3 alcohol sensor data into CAN message
 *
 * Message format (8 bytes):
 *   Byte 0-1: Raw ADC (uint16_t, 0-4095)
 *   Byte 2-3: Rs/R0 ratio (uint16_t, ratio × 1000)
 *   Byte 4-5: PPM estimate (uint16_t)
 *   Byte 6:   Status
 *   Byte 7:   Sequence counter
 *
 * @param[in] data Sensor data structure
 * @param[in] sequence Sequence counter value
 * @param[out] msg CAN message to populate
 */
void can_format_mq3_message(const sensor_data_t *data, uint8_t sequence, twai_message_t *msg);

/**
 * @brief Format system status message
 *
 * Message format (8 bytes):
 *   Byte 0:   Active sensors bitmask
 *   Byte 1:   Free heap (KB)
 *   Byte 2-5: Uptime (uint32_t, seconds)
 *   Byte 6-7: Sequence counter
 *
 * @param[in] active_sensors Bitmask of active sensors
 * @param[in] free_heap_kb Free heap in kilobytes
 * @param[in] uptime_sec System uptime in seconds
 * @param[in] sequence Sequence counter value
 * @param[out] msg CAN message to populate
 */
void can_format_status_message(uint8_t active_sensors, uint8_t free_heap_kb,
                               uint32_t uptime_sec, uint16_t sequence,
                               twai_message_t *msg);

/**
 * @brief Format INFO_RESPONSE message
 *
 * Message format (8 bytes):
 *   Byte 0:   Current node ID
 *   Byte 1:   Firmware version major
 *   Byte 2:   Firmware version minor
 *   Byte 3:   Firmware version patch
 *   Byte 4:   Sensor flags (bit 0: ALS, bit 1: BME680, etc.)
 *   Byte 5:   ALS type (0=none, 1=VEML7700, 2=OPT4001)
 *   Byte 6:   Status flags (bit 0: transmitting)
 *   Byte 7:   Reserved
 *
 * @param[in] node_id Current node ID (0-5)
 * @param[in] sensor_flags Bitmask of detected sensors
 * @param[in] als_type ALS sensor type (ALS_TYPE_*)
 * @param[in] status_flags Current status flags
 * @param[out] msg CAN message to populate
 */
void can_format_info_response(uint8_t node_id, uint8_t sensor_flags,
                              uint8_t als_type, uint8_t status_flags,
                              twai_message_t *msg);

/**
 * @brief Format PONG discovery response message
 *
 * Message format (8 bytes):
 *   Byte 0:   Current node ID
 *   Byte 1-7: Reserved (0x00)
 *
 * @param[in] node_id Current node ID (0-5)
 * @param[out] msg CAN message to populate
 */
void can_format_pong_response(uint8_t node_id, twai_message_t *msg);

/**
 * @brief Set CAN message identifier with node ID offset
 *
 * Updates the identifier of an existing CAN message to use the specified
 * node's address range.
 *
 * @param[in,out] msg CAN message to update
 * @param[in] node_id Node ID (0-5)
 * @param[in] offset Message type offset (CAN_MSG_OFFSET_*)
 */
void can_set_msg_id(twai_message_t *msg, uint8_t node_id, uint8_t offset);

#endif // CAN_PROTOCOL_H
