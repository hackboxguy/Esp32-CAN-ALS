/* can_protocol.c - CAN Message Protocol Implementation */

#include "can_protocol.h"
#include <string.h>

/* ======================== Helper Functions ======================== */

static uint16_t calculate_checksum(const uint8_t *data, size_t len) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

/* ======================== Message Formatting Functions ======================== */

void can_format_veml7700_message(const sensor_data_t *data, uint8_t sequence, twai_message_t *msg) {
    memset(msg, 0, sizeof(twai_message_t));

    msg->identifier = ID_SENSOR_VEML7700;
    msg->flags = TWAI_MSG_FLAG_NONE;
    msg->data_length_code = 8;

    /* Lux value (uint24_t, 3 bytes, 0-16,777,215 lux) */
    uint32_t lux_int = (uint32_t)(data->data.veml.lux);
    if (lux_int > 0xFFFFFF) lux_int = 0xFFFFFF;  /* Clamp to 24-bit max */

    msg->data[0] = lux_int & 0xFF;          /* Lux low byte */
    msg->data[1] = (lux_int >> 8) & 0xFF;   /* Lux mid byte */
    msg->data[2] = (lux_int >> 16) & 0xFF;  /* Lux high byte */

    /* Status, sequence, config index */
    msg->data[3] = data->status;
    msg->data[4] = sequence;
    msg->data[5] = data->data.veml.config_idx;

    /* Checksum (bytes 0-5) */
    uint16_t checksum = calculate_checksum(msg->data, 6);
    msg->data[6] = checksum & 0xFF;
    msg->data[7] = (checksum >> 8) & 0xFF;
}

void can_format_bme_env_message(const sensor_data_t *data, twai_message_t *msg) {
    memset(msg, 0, sizeof(twai_message_t));

    msg->identifier = ID_SENSOR_BME_ENV;
    msg->flags = TWAI_MSG_FLAG_NONE;
    msg->data_length_code = 8;

    /* Temperature (int16_t, °C × 100) */
    int16_t temp_scaled = (int16_t)(data->data.bme.temperature * 100);
    msg->data[0] = temp_scaled & 0xFF;
    msg->data[1] = (temp_scaled >> 8) & 0xFF;

    /* Humidity (uint8_t, %RH) */
    msg->data[2] = (uint8_t)data->data.bme.humidity;

    /* Pressure (uint16_t, hPa × 10) */
    uint16_t press_scaled = (uint16_t)(data->data.bme.pressure * 10);
    msg->data[3] = press_scaled & 0xFF;
    msg->data[4] = (press_scaled >> 8) & 0xFF;

    /* Status */
    msg->data[5] = data->status;

    /* Checksum (bytes 0-5) */
    uint16_t checksum = calculate_checksum(msg->data, 6);
    msg->data[6] = checksum & 0xFF;
    msg->data[7] = (checksum >> 8) & 0xFF;
}

void can_format_bme_aiq_message(const sensor_data_t *data, twai_message_t *msg) {
    memset(msg, 0, sizeof(twai_message_t));

    msg->identifier = ID_SENSOR_BME_AIQ;
    msg->flags = TWAI_MSG_FLAG_NONE;
    msg->data_length_code = 8;

    /* IAQ (uint16_t, 0-500) */
    msg->data[0] = data->data.bme.iaq & 0xFF;
    msg->data[1] = (data->data.bme.iaq >> 8) & 0xFF;

    /* Accuracy (uint8_t, 0-3) */
    msg->data[2] = data->data.bme.accuracy;

    /* CO2 equivalent (uint16_t, ppm) */
    msg->data[3] = data->data.bme.co2_equiv & 0xFF;
    msg->data[4] = (data->data.bme.co2_equiv >> 8) & 0xFF;

    /* Breath VOC (uint16_t, ppm) */
    msg->data[5] = data->data.bme.breath_voc & 0xFF;
    msg->data[6] = (data->data.bme.breath_voc >> 8) & 0xFF;

    /* Status */
    msg->data[7] = data->status;
}

void can_format_ld2410_message(const sensor_data_t *data, twai_message_t *msg) {
    memset(msg, 0, sizeof(twai_message_t));

    msg->identifier = ID_SENSOR_LD2410;
    msg->flags = TWAI_MSG_FLAG_NONE;
    msg->data_length_code = 8;

    /* Presence (bool as uint8_t) */
    msg->data[0] = data->data.ld2410.presence_detected ? 1 : 0;

    /* Distance (uint16_t, cm) */
    msg->data[1] = data->data.ld2410.distance_cm & 0xFF;
    msg->data[2] = (data->data.ld2410.distance_cm >> 8) & 0xFF;

    /* Energy levels */
    msg->data[3] = data->data.ld2410.move_energy;
    msg->data[4] = data->data.ld2410.static_energy;

    /* Status */
    msg->data[5] = data->status;

    /* Checksum (bytes 0-5) */
    uint16_t checksum = calculate_checksum(msg->data, 6);
    msg->data[6] = checksum & 0xFF;
    msg->data[7] = (checksum >> 8) & 0xFF;
}

void can_format_mq3_message(const sensor_data_t *data, uint8_t sequence, twai_message_t *msg) {
    memset(msg, 0, sizeof(twai_message_t));

    msg->identifier = ID_SENSOR_MQ3;
    msg->flags = TWAI_MSG_FLAG_NONE;
    msg->data_length_code = 8;

    /* Raw ADC (uint16_t, 0-4095) */
    msg->data[0] = data->data.mq3.raw_adc & 0xFF;
    msg->data[1] = (data->data.mq3.raw_adc >> 8) & 0xFF;

    /* Rs/R0 ratio (uint16_t, ratio × 1000) */
    uint16_t ratio_scaled = (uint16_t)(data->data.mq3.ratio * 1000);
    msg->data[2] = ratio_scaled & 0xFF;
    msg->data[3] = (ratio_scaled >> 8) & 0xFF;

    /* PPM estimate (uint16_t) */
    msg->data[4] = data->data.mq3.ppm_estimate & 0xFF;
    msg->data[5] = (data->data.mq3.ppm_estimate >> 8) & 0xFF;

    /* Status and sequence */
    msg->data[6] = data->status;
    msg->data[7] = sequence;
}

void can_format_status_message(uint8_t active_sensors, uint8_t free_heap_kb,
                               uint32_t uptime_sec, uint16_t sequence,
                               twai_message_t *msg) {
    memset(msg, 0, sizeof(twai_message_t));

    msg->identifier = ID_SENSOR_STATUS;
    msg->flags = TWAI_MSG_FLAG_NONE;
    msg->data_length_code = 8;

    /* Active sensors bitmask */
    msg->data[0] = active_sensors;

    /* Free heap in KB */
    msg->data[1] = free_heap_kb;

    /* Uptime in seconds (uint32_t, little-endian) */
    msg->data[2] = uptime_sec & 0xFF;
    msg->data[3] = (uptime_sec >> 8) & 0xFF;
    msg->data[4] = (uptime_sec >> 16) & 0xFF;
    msg->data[5] = (uptime_sec >> 24) & 0xFF;

    /* Sequence counter (uint16_t) */
    msg->data[6] = sequence & 0xFF;
    msg->data[7] = (sequence >> 8) & 0xFF;
}

void can_format_info_response(uint8_t node_id, uint8_t sensor_flags,
                              uint8_t als_type, uint8_t status_flags,
                              uint8_t partition_info, twai_message_t *msg) {
    memset(msg, 0, sizeof(twai_message_t));

    msg->identifier = CAN_MSG_ID(node_id, CAN_MSG_OFFSET_INFO_RESP);
    msg->flags = TWAI_MSG_FLAG_NONE;
    msg->data_length_code = 8;

    msg->data[0] = node_id;
    msg->data[1] = FIRMWARE_VERSION_MAJOR;
    msg->data[2] = FIRMWARE_VERSION_MINOR;
    msg->data[3] = FIRMWARE_VERSION_PATCH;
    msg->data[4] = sensor_flags;
    msg->data[5] = als_type;
    msg->data[6] = status_flags;
    msg->data[7] = partition_info;  /* Partition type (bits 0-2) + OTA state (bits 4-6) */
}

void can_format_pong_response(uint8_t node_id, twai_message_t *msg) {
    memset(msg, 0, sizeof(twai_message_t));

    msg->identifier = CAN_MSG_ID(node_id, CAN_MSG_OFFSET_PONG);
    msg->flags = TWAI_MSG_FLAG_NONE;
    msg->data_length_code = 8;

    msg->data[0] = node_id;
    /* Bytes 1-7 reserved (already zeroed by memset) */
}

void can_set_msg_id(twai_message_t *msg, uint8_t node_id, uint8_t offset) {
    msg->identifier = CAN_MSG_ID(node_id, offset);
}
