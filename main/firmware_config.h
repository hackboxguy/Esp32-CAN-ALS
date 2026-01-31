/* firmware_config.h - Firmware Configuration
 *
 * Version and feature configuration for ESP32-CAN-Sensor firmware.
 */

#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

/* Feature flags - all sensors always enabled */
#define SENSORS_ENABLED     1
#define AUTO_START_TX       1

/* Firmware version (can be overridden via CMAKE -D flags) */
#ifndef FIRMWARE_VERSION_MAJOR
#define FIRMWARE_VERSION_MAJOR  1
#endif

#ifndef FIRMWARE_VERSION_MINOR
#define FIRMWARE_VERSION_MINOR  0
#endif

#ifndef FIRMWARE_VERSION_PATCH
#define FIRMWARE_VERSION_PATCH  0
#endif

#endif /* FIRMWARE_CONFIG_H */
