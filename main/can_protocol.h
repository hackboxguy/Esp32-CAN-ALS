/* can_protocol.h - CAN Message Protocol Definitions and Formatting */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <driver/twai.h>
#include "sensor_common.h"

/* ======================== CAN Message IDs ======================== */

/* Control Messages (RX - from master to ESP32) */
#define ID_MASTER_STOP_CMD      0x0A0  /* Stop sensor transmission */
#define ID_MASTER_START_CMD     0x0A1  /* Start sensor transmission */
#define ID_MASTER_SHUTDOWN_CMD  0x0A8  /* Graceful shutdown (save state, keep running) */
#define ID_MASTER_REBOOT_CMD    0x0A9  /* Save state and reboot ESP32 */
#define ID_MASTER_FACTORY_RST   0x0AA  /* Factory reset (clear calibration, reboot) */

/* Sensor Data Messages (TX - from ESP32 to master) */
#define ID_SENSOR_VEML7700      0x0A2  /* Ambient light (1 Hz) */
#define ID_SENSOR_BME_ENV       0x0A3  /* BME680/688 environmental (0.33 Hz) */
#define ID_SENSOR_BME_AIQ       0x0A4  /* BME680/688 air quality (0.33 Hz) */
#define ID_SENSOR_LD2410        0x0A5  /* Presence detection (10 Hz) */
#define ID_SENSOR_MQ3           0x0A6  /* Alcohol/VOC (1 Hz) */
#define ID_SENSOR_STATUS        0x0A7  /* System status (0.1 Hz) */

/* Legacy alias for backward compatibility */
#define ID_MASTER_DATA          ID_SENSOR_VEML7700

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

#endif // CAN_PROTOCOL_H
