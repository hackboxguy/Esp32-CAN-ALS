/* sensor_common.h - Shared data structures for multi-sensor system */

#ifndef SENSOR_COMMON_H
#define SENSOR_COMMON_H

#include <stdint.h>
#include <stdbool.h>

/* Sensor identifiers */
typedef enum {
    SENSOR_VEML7700 = 0,
    SENSOR_BME680,
    SENSOR_LD2410,
    SENSOR_MQ3,
    NUM_SENSORS
} sensor_id_t;

/* Sensor status codes */
#define SENSOR_STATUS_OK            0x00
#define SENSOR_STATUS_ERROR         0x01
#define SENSOR_STATUS_CALIBRATING   0x02

/* Sensor data structure (union for all sensor types) */
typedef struct {
    sensor_id_t sensor_id;
    uint32_t timestamp_ms;
    uint8_t status;

    union {
        /* VEML7700 - Ambient light sensor */
        struct {
            float lux;
            uint8_t config_idx;
        } veml;

        /* BME680/BME688 - Environmental sensor */
        struct {
            float temperature;     // °C
            float humidity;        // %RH
            float pressure;        // hPa
            uint16_t iaq;          // 0-500 (air quality index)
            uint8_t accuracy;      // 0-3 (BSEC calibration)
            uint16_t co2_equiv;    // ppm
            uint16_t breath_voc;   // ppm
        } bme;

        /* HLK-LD2410 - Presence radar */
        struct {
            bool presence_detected;
            uint16_t distance_cm;
            uint8_t move_energy;
            uint8_t static_energy;
        } ld2410;

        /* MQ-3 - Alcohol/VOC gas sensor */
        struct {
            uint16_t raw_adc;      // 0-4095
            float resistance;      // Rs in kΩ
            float ratio;           // Rs/R0
            uint16_t ppm_estimate; // Rough alcohol estimate
        } mq3;
    } data;
} sensor_data_t;

#endif // SENSOR_COMMON_H
