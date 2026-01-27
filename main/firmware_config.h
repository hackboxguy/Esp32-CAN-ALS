/* firmware_config.h - Firmware Type Configuration
 *
 * Defines FACTORY vs MAIN firmware build configuration.
 * Set via CMake: -DFIRMWARE_TYPE=FACTORY or -DFIRMWARE_TYPE=MAIN
 */

#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

/* Firmware type values */
#define FIRMWARE_TYPE_FACTORY   0
#define FIRMWARE_TYPE_MAIN      1

/* Default to MAIN if not specified via CMAKE */
#ifndef FIRMWARE_TYPE
#define FIRMWARE_TYPE   FIRMWARE_TYPE_MAIN
#endif

/* Convenience macros for conditional compilation */
#define IS_FACTORY_FIRMWARE (FIRMWARE_TYPE == FIRMWARE_TYPE_FACTORY)
#define IS_MAIN_FIRMWARE    (FIRMWARE_TYPE == FIRMWARE_TYPE_MAIN)

/* Feature flags derived from firmware type */
#if IS_MAIN_FIRMWARE
#define SENSORS_ENABLED     1
#define AUTO_START_TX       1
#else
#define SENSORS_ENABLED     0
#define AUTO_START_TX       0
#endif

/* Firmware type string for logging */
#if IS_MAIN_FIRMWARE
#define FIRMWARE_TYPE_STR   "MAIN"
#else
#define FIRMWARE_TYPE_STR   "FACTORY"
#endif

#endif /* FIRMWARE_CONFIG_H */
