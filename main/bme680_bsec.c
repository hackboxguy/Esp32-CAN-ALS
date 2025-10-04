/* bme680_bsec.c - BME680/BME688 Environmental Sensor Driver with BSEC
 *
 * Unified driver supporting both BME680 and BME688 with Bosch BSEC library.
 * Provides temperature, humidity, pressure, and air quality measurements.
 */

#include "bme680_bsec.h"

#ifdef CONFIG_BME680_ENABLED

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <driver/i2c.h>

/* BSEC library headers (from components/bsec) */
#include "bsec_interface.h"
#include "bsec_datatypes.h"

/* BME68x sensor API (from components/bsec) */
#include "bme68x.h"
#include "bme68x_defs.h"

/* BSEC config (from components/bsec/config) */
#include "bsec_iaq.h"

static const char *TAG = "BME68x";

/* ======================== Configuration ======================== */

/* I2C configuration (shared with ALS sensors) */
#define BME68X_I2C_NUM          I2C_NUM_0
#define BME68X_I2C_SDA_GPIO     GPIO_NUM_6
#define BME68X_I2C_SCL_GPIO     GPIO_NUM_7
#define BME68X_I2C_FREQ_HZ      100000  /* 100 kHz (BME68x max: 3.4 MHz) */

/* I2C address from Kconfig */
#ifndef CONFIG_BME680_I2C_ADDR
#define BME68X_I2C_ADDR         0x76    /* Default if Kconfig not available */
#else
#define BME68X_I2C_ADDR         CONFIG_BME680_I2C_ADDR
#endif

/* BSEC configuration */
#define BSEC_SAMPLE_RATE        BSEC_SAMPLE_RATE_LP  /* Low Power: 3 seconds */
#define BSEC_MAX_STATE_SIZE     BSEC_MAX_PROPERTY_BLOB_SIZE  /* ~140 bytes */
#define NVS_NAMESPACE           "bsec_state"
#define NVS_KEY_CALIBRATION     "calibration"
#define NVS_KEY_TIMESTAMP       "timestamp"

/* State save interval from Kconfig (default 4 hours) */
#ifndef CONFIG_BME680_SAVE_STATE_INTERVAL
#define STATE_SAVE_INTERVAL_SEC (4 * 3600)
#else
#define STATE_SAVE_INTERVAL_SEC (CONFIG_BME680_SAVE_STATE_INTERVAL * 3600)
#endif

/* ======================== Global State ======================== */

static struct {
    struct bme68x_dev sensor;           /* BME68x device structure */

    bool initialized;                   /* Driver initialized flag */
    bool is_bme688;                     /* true=BME688, false=BME680 */
    uint8_t i2c_address;               /* Actual I2C address */

    bsec_bme_settings_t bsec_settings;  /* BSEC sensor settings */
    uint8_t bsec_state[BSEC_MAX_STATE_SIZE]; /* BSEC calibration state */

    uint64_t last_state_save_time_us;   /* Last NVS save timestamp */

} g_bme68x = {
    .initialized = false,
    .is_bme688 = false,
    .i2c_address = BME68X_I2C_ADDR,
};

/* ======================== I2C Wrapper Functions ======================== */

/**
 * @brief I2C read wrapper for BME68x API
 */
static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *data,
                                             uint32_t len, void *intf_ptr) {
    /* Create I2C command link */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    /* Write register address */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_bme68x.i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    /* Read data */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_bme68x.i2c_address << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    /* Execute transaction */
    esp_err_t ret = i2c_master_cmd_begin(BME68X_I2C_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

/**
 * @brief I2C write wrapper for BME68x API
 */
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *data,
                                              uint32_t len, void *intf_ptr) {
    /* Create I2C command link */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    /* Write register address + data */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_bme68x.i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);

    /* Execute transaction */
    esp_err_t ret = i2c_master_cmd_begin(BME68X_I2C_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

/**
 * @brief Delay function wrapper for BME68x API
 */
static void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    esp_rom_delay_us(period);
}

/* ======================== NVS State Management ======================== */

/**
 * @brief Load BSEC calibration state from NVS
 */
static esp_err_t bsec_load_state_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace not found, starting with fresh calibration");
        return ESP_ERR_NVS_NOT_FOUND;
    }

    /* Read calibration blob */
    size_t required_size = BSEC_MAX_STATE_SIZE;
    ret = nvs_get_blob(nvs_handle, NVS_KEY_CALIBRATION, g_bme68x.bsec_state,
                       &required_size);

    if (ret == ESP_OK) {
        /* Restore state to BSEC library */
        static uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];  /* Static to avoid stack overflow (4KB) */
        bsec_library_return_t bsec_status = bsec_set_state(g_bme68x.bsec_state,
                                                            required_size,
                                                            work_buffer,
                                                            BSEC_MAX_WORKBUFFER_SIZE);

        if (bsec_status == BSEC_OK) {
            /* Read state timestamp */
            uint64_t save_timestamp = 0;
            nvs_get_u64(nvs_handle, NVS_KEY_TIMESTAMP, &save_timestamp);

            uint64_t age_sec = (esp_timer_get_time() - save_timestamp) / 1000000;
            ESP_LOGI(TAG, "Loaded BSEC state from NVS (age: %llu hours)", age_sec / 3600);
        } else {
            ESP_LOGW(TAG, "Failed to restore BSEC state (error %d)", bsec_status);
            ret = ESP_FAIL;
        }
    } else {
        ESP_LOGW(TAG, "No calibration data in NVS");
    }

    nvs_close(nvs_handle);
    return ret;
}

/**
 * @brief Save BSEC calibration state to NVS
 */
esp_err_t bme680_save_state(void) {
    if (!g_bme68x.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret;

    /* Get current BSEC state */
    uint32_t n_serialized_state = BSEC_MAX_STATE_SIZE;
    bsec_library_return_t bsec_status = bsec_get_state(0, g_bme68x.bsec_state,
                                                        BSEC_MAX_STATE_SIZE,
                                                        g_bme68x.bsec_state,
                                                        0,
                                                        &n_serialized_state);

    if (bsec_status != BSEC_OK) {
        ESP_LOGE(TAG, "Failed to get BSEC state (error %d)", bsec_status);
        return ESP_FAIL;
    }

    /* Open NVS */
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
        return ret;
    }

    /* Write calibration blob */
    ret = nvs_set_blob(nvs_handle, NVS_KEY_CALIBRATION, g_bme68x.bsec_state,
                       n_serialized_state);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write calibration to NVS");
        nvs_close(nvs_handle);
        return ret;
    }

    /* Write timestamp */
    uint64_t now = esp_timer_get_time();
    ret = nvs_set_u64(nvs_handle, NVS_KEY_TIMESTAMP, now);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write timestamp to NVS");
        nvs_close(nvs_handle);
        return ret;
    }

    /* Commit changes */
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        g_bme68x.last_state_save_time_us = now;
        ESP_LOGI(TAG, "BSEC state saved to NVS (%d bytes)", n_serialized_state);
    }

    return ret;
}

/**
 * @brief Reset BSEC calibration state
 */
esp_err_t bme680_reset_calibration(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Erase calibration data */
    nvs_erase_key(nvs_handle, NVS_KEY_CALIBRATION);
    nvs_erase_key(nvs_handle, NVS_KEY_TIMESTAMP);
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BSEC calibration state reset");
    }

    return ret;
}

/* ======================== BSEC Integration ======================== */

/**
 * @brief Initialize BSEC library and configure outputs
 */
static esp_err_t bsec_init_library(void) {
    bsec_library_return_t bsec_status;

    /* Initialize BSEC library */
    bsec_status = bsec_init();
    if (bsec_status != BSEC_OK) {
        ESP_LOGE(TAG, "Failed to initialize BSEC (error %d)", bsec_status);
        return ESP_FAIL;
    }

    /* Get BSEC version */
    bsec_version_t version;
    bsec_get_version(&version);
    ESP_LOGI(TAG, "BSEC version: %d.%d.%d.%d",
             version.major, version.minor, version.major_bugfix, version.minor_bugfix);

    /* Load BSEC configuration for BME680 @ 3.3V, LP mode (3s), 4-day calibration */
    static uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];  /* Static to avoid stack overflow (4KB) */
    bsec_status = bsec_set_configuration(bsec_config_iaq, sizeof(bsec_config_iaq),
                                          work_buffer, sizeof(work_buffer));
    if (bsec_status != BSEC_OK) {
        ESP_LOGE(TAG, "Failed to set BSEC configuration (error %d)", bsec_status);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BSEC configuration loaded (3.3V, LP/3s, 4-day)");

    /* Configure requested virtual sensors for LP mode (3 seconds)
     * IAQ config may not support RAW outputs - try HEAT_COMPENSATED instead
     * Based on BSEC_OUTPUT_INCLUDED bitfield: 0x1279ef */
    //bsec_sensor_configuration_t requested_virtual_sensors[] = {
        /* Try heat-compensated outputs which are definitely supported */
    //    {BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE, BSEC_SAMPLE_RATE_LP},  // ID=13
    //    {BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY, BSEC_SAMPLE_RATE_LP},      // ID=14
    //    {BSEC_OUTPUT_STATIC_IAQ, BSEC_SAMPLE_RATE_LP},                            // ID=2
    //};
    
    //bsec_sensor_configuration_t requested_virtual_sensors[] = {
        /* RAW sensor outputs */
    //    {BSEC_OUTPUT_RAW_TEMPERATURE, BSEC_SAMPLE_RATE_LP},
    //    {BSEC_OUTPUT_RAW_PRESSURE, BSEC_SAMPLE_RATE_LP},
    //    {BSEC_OUTPUT_RAW_HUMIDITY, BSEC_SAMPLE_RATE_LP},
    //    {BSEC_OUTPUT_RAW_GAS, BSEC_SAMPLE_RATE_LP},
        /* Air quality outputs (IAQ config supports STATIC_IAQ + derivatives) */
    //    {BSEC_OUTPUT_STATIC_IAQ, BSEC_SAMPLE_RATE_LP},
    //    {BSEC_OUTPUT_CO2_EQUIVALENT, BSEC_SAMPLE_RATE_LP},
    //    {BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, BSEC_SAMPLE_RATE_LP},
        /* Heat compensated environmental outputs */
    //    {BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE, BSEC_SAMPLE_RATE_LP},
    //    {BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY, BSEC_SAMPLE_RATE_LP},
    //};

    bsec_sensor_configuration_t requested_virtual_sensors[] = {
        /* RAW sensor outputs */
        {BSEC_OUTPUT_RAW_TEMPERATURE, BSEC_SAMPLE_RATE_LP},
        {BSEC_OUTPUT_RAW_PRESSURE, BSEC_SAMPLE_RATE_LP},
        {BSEC_OUTPUT_RAW_HUMIDITY, BSEC_SAMPLE_RATE_LP},
        {BSEC_OUTPUT_RAW_GAS, BSEC_SAMPLE_RATE_LP},
        /* Air quality outputs (IAQ config supports STATIC_IAQ + derivatives) */
        {BSEC_OUTPUT_STATIC_IAQ, BSEC_SAMPLE_RATE_LP},
        {BSEC_OUTPUT_CO2_EQUIVALENT, BSEC_SAMPLE_RATE_LP},
        {BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, BSEC_SAMPLE_RATE_LP},
        /* Heat compensated environmental outputs */
        {BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE, BSEC_SAMPLE_RATE_LP},
        {BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY, BSEC_SAMPLE_RATE_LP},
    };

    uint8_t n_requested = sizeof(requested_virtual_sensors) /
                         sizeof(requested_virtual_sensors[0]);

    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required = BSEC_MAX_PHYSICAL_SENSOR;  /* Must indicate available array size */

    bsec_status = bsec_update_subscription(requested_virtual_sensors, n_requested,
                                           required_sensor_settings, &n_required);

    if (bsec_status < BSEC_OK) {
        /* Negative values are errors */
        ESP_LOGE(TAG, "Failed to configure BSEC subscriptions (error %d)", bsec_status);
        return ESP_FAIL;
    } else if (bsec_status > BSEC_OK) {
        /* Positive values are warnings - log but continue */
        ESP_LOGW(TAG, "BSEC subscription warning %d (some outputs may not be available)", bsec_status);
    }

    ESP_LOGI(TAG, "BSEC configured: %d virtual sensors requested, %d physical sensors required",
             n_requested, n_required);

    /* Log what physical sensors BSEC is requesting */
    ESP_LOGI(TAG, "Physical sensor requirements:");
    for (int i = 0; i < n_required; i++) {
        ESP_LOGI(TAG, "  [%d] sensor_id=%d, sample_rate=%.4f",
                 i, required_sensor_settings[i].sensor_id,
                 required_sensor_settings[i].sample_rate);
    }

    /* Try to load calibration state from NVS */
    bsec_load_state_from_nvs();

    /* Initialize BSEC timing by calling sensor_control once */
    int64_t timestamp_ns = esp_timer_get_time() * 1000;
    bsec_sensor_control(timestamp_ns, &g_bme68x.bsec_settings);
    ESP_LOGI(TAG, "Initial BSEC next_call: %lld ms", g_bme68x.bsec_settings.next_call / 1000000);

    return ESP_OK;
}

/* ======================== Sensor Initialization ======================== */

/**
 * @brief Initialize BME680/BME688 sensor and BSEC library
 *
 * NOTE: I2C bus must already be initialized by als_init() before calling this
 */
esp_err_t bme680_init(void) {
    esp_err_t ret;
    int8_t bme_status;

    if (g_bme68x.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* NOTE: I2C bus already initialized by als_init() in als_driver.c */
    /* We use the same I2C bus (I2C_NUM_0) with the old driver API */

    /* Auto-detect BME680/688 at 0x76 or 0x77 */
    ESP_LOGI(TAG, "Auto-detecting BME680/688 sensor...");

    uint8_t addresses_to_try[] = {BME68X_I2C_ADDR, 0x76, 0x77};  /* Try Kconfig addr first, then both */
    bool sensor_found = false;

    for (int i = 0; i < 3; i++) {
        /* Skip duplicate addresses */
        if (i > 0 && addresses_to_try[i] == addresses_to_try[0]) continue;

        g_bme68x.i2c_address = addresses_to_try[i];
        ESP_LOGI(TAG, "Probing I2C address 0x%02X...", g_bme68x.i2c_address);

        /* Test I2C communication by trying to read chip ID register (0xD0) */
        uint8_t chip_id = 0;
        int8_t test_result = bme68x_i2c_read(0xD0, &chip_id, 1, NULL);

        if (test_result == BME68X_OK && chip_id == 0x61) {
            ESP_LOGI(TAG, "BME680/688 detected at address 0x%02X (chip_id=0x61)", g_bme68x.i2c_address);
            sensor_found = true;
            break;
        } else {
            ESP_LOGD(TAG, "No response at 0x%02X (result=%d, chip_id=0x%02X)",
                     g_bme68x.i2c_address, test_result, chip_id);
        }
    }

    if (!sensor_found) {
        ESP_LOGW(TAG, "BME680/688 not found at any address (tried 0x76, 0x77)");
        return ESP_ERR_NOT_FOUND;
    }

    /* Initialize BME68x sensor API */
    g_bme68x.sensor.intf = BME68X_I2C_INTF;
    g_bme68x.sensor.read = bme68x_i2c_read;
    g_bme68x.sensor.write = bme68x_i2c_write;
    g_bme68x.sensor.delay_us = bme68x_delay_us;
    g_bme68x.sensor.intf_ptr = NULL;
    g_bme68x.sensor.amb_temp = 25; /* Assumed ambient temperature for delay calc */

    bme_status = bme68x_init(&g_bme68x.sensor);
    if (bme_status != BME68X_OK) {
        ESP_LOGE(TAG, "BME68x init failed (error %d) at address 0x%02X",
                 bme_status, g_bme68x.i2c_address);
        return ESP_ERR_NOT_FOUND;
    }

    /* Check sensor variant (BME680 vs BME688) */
    uint8_t variant_id = g_bme68x.sensor.variant_id;
    g_bme68x.is_bme688 = (variant_id == BME68X_VARIANT_GAS_HIGH);

    ESP_LOGI(TAG, "Detected: %s (chip_id=0x%02X, variant=0x%02X)",
             g_bme68x.is_bme688 ? "BME688" : "BME680",
             g_bme68x.sensor.chip_id, variant_id);

    /* Initialize BSEC library */
    ret = bsec_init_library();
    if (ret != ESP_OK) {
        return ret;
    }

    g_bme68x.initialized = true;
    g_bme68x.last_state_save_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Initialization complete");
    return ESP_OK;
}

/* ======================== Sensor Reading ======================== */

/**
 * @brief Read sensor and process with BSEC
 */
esp_err_t bme680_read(bme68x_data_t *data) {
    if (!g_bme68x.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int8_t bme_status;
    bsec_library_return_t bsec_status;

    /* Get current timestamp in nanoseconds (BSEC requirement) */
    int64_t timestamp_ns = esp_timer_get_time() * 1000;

    /* Step 1: Call BSEC to get sensor control settings */
    bsec_status = bsec_sensor_control(timestamp_ns, &g_bme68x.bsec_settings);

    ESP_LOGI(TAG, "bsec_sensor_control: status=%d, trigger=%d, op_mode=%d, next_call=%lld ns",
             bsec_status, g_bme68x.bsec_settings.trigger_measurement,
             g_bme68x.bsec_settings.op_mode, g_bme68x.bsec_settings.next_call);

    ESP_LOGI(TAG, "BSEC settings: run_gas=%d, temp_os=%d, press_os=%d, hum_os=%d",
             g_bme68x.bsec_settings.run_gas,
             g_bme68x.bsec_settings.temperature_oversampling,
             g_bme68x.bsec_settings.pressure_oversampling,
             g_bme68x.bsec_settings.humidity_oversampling);

    if (bsec_status < BSEC_OK) {
        /* Negative values are errors */
        ESP_LOGE(TAG, "bsec_sensor_control failed (error %d)", bsec_status);
        return ESP_FAIL;
    } else if (bsec_status > BSEC_OK) {
        /* Positive values are warnings (e.g., timing violation) - log but continue */
        ESP_LOGW(TAG, "bsec_sensor_control warning %d", bsec_status);
    }

    /* Check if BSEC wants a measurement at this time */
    if (g_bme68x.bsec_settings.trigger_measurement == 0) {
        /* BSEC doesn't need measurement yet - too early */
        ESP_LOGI(TAG, "BSEC trigger=0 - no measurement needed (next call: %lld ns)",
                 g_bme68x.bsec_settings.next_call);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "BSEC triggering measurement: heater=%d°C, duration=%dms",
             g_bme68x.bsec_settings.heater_temperature,
             g_bme68x.bsec_settings.heater_duration);

    /* Step 2: Configure BME68x sensor according to BSEC requirements */
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;

    /* Set temperature/pressure/humidity oversampling */
    bme_status = bme68x_get_conf(&conf, &g_bme68x.sensor);
    if (bme_status != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to get sensor config (error %d)", bme_status);
        return ESP_FAIL;
    }

    /* Use BSEC-provided oversampling settings (CRITICAL!) */
    conf.os_hum = g_bme68x.bsec_settings.humidity_oversampling;
    conf.os_temp = g_bme68x.bsec_settings.temperature_oversampling;
    conf.os_pres = g_bme68x.bsec_settings.pressure_oversampling;
    conf.filter = BME68X_FILTER_OFF; /* IIR filter coefficient */
    conf.odr = BME68X_ODR_NONE;    /* Output data rate (not used in forced mode) */

    bme_status = bme68x_set_conf(&conf, &g_bme68x.sensor);
    if (bme_status != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set sensor config (error %d)", bme_status);
        return ESP_FAIL;
    }

    /* Set heater configuration from BSEC */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = g_bme68x.bsec_settings.heater_temperature;
    heatr_conf.heatr_dur = g_bme68x.bsec_settings.heater_duration;

    bme_status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &g_bme68x.sensor);
    if (bme_status != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set heater config (error %d)", bme_status);
        return ESP_FAIL;
    }

    /* Step 3: Trigger forced mode measurement */
    bme_status = bme68x_set_op_mode(BME68X_FORCED_MODE, &g_bme68x.sensor);
    if (bme_status != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to set forced mode (error %d)", bme_status);
        return ESP_FAIL;
    }

    /* Step 4: Wait for measurement to complete */
    uint32_t delay_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &g_bme68x.sensor);
    delay_us += heatr_conf.heatr_dur * 1000; /* Add heater duration */
    g_bme68x.sensor.delay_us(delay_us, g_bme68x.sensor.intf_ptr);

    /* Poll until measurement ready */
    uint8_t n_fields = 0;
    struct bme68x_data sensor_data[3];  /* BME688 can return up to 3 fields */

    bme_status = bme68x_get_data(BME68X_FORCED_MODE, sensor_data, &n_fields, &g_bme68x.sensor);
    if (bme_status != BME68X_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data (error %d)", bme_status);
        return ESP_FAIL;
    }

    if (n_fields == 0) {
        ESP_LOGW(TAG, "No new data available");
        return ESP_ERR_NOT_FOUND;
    }

    /* Use first field (forced mode typically returns 1 field) */
    struct bme68x_data *field = &sensor_data[0];

    /* Check data validity flags */
    if (!(field->status & BME68X_NEW_DATA_MSK)) {
        ESP_LOGW(TAG, "Data not new");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Step 5: Prepare BSEC inputs from raw sensor data */
    bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_inputs = 0;

    /* Temperature input */
    bsec_inputs[n_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
    bsec_inputs[n_inputs].signal = field->temperature;
    bsec_inputs[n_inputs].time_stamp = timestamp_ns;
    n_inputs++;

    /* Humidity input */
    bsec_inputs[n_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
    bsec_inputs[n_inputs].signal = field->humidity;
    bsec_inputs[n_inputs].time_stamp = timestamp_ns;
    n_inputs++;

    /* Pressure input */
    bsec_inputs[n_inputs].sensor_id = BSEC_INPUT_PRESSURE;
    bsec_inputs[n_inputs].signal = field->pressure;
    bsec_inputs[n_inputs].time_stamp = timestamp_ns;
    n_inputs++;

    /* Gas resistance input (if valid) */
    if (field->status & BME68X_GASM_VALID_MSK) {
        bsec_inputs[n_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        bsec_inputs[n_inputs].signal = field->gas_resistance;
        bsec_inputs[n_inputs].time_stamp = timestamp_ns;
        n_inputs++;
    }

    /* Step 6: Process data with BSEC algorithm */
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t n_outputs = BSEC_NUMBER_OUTPUTS;

    ESP_LOGI(TAG, "Calling bsec_do_steps with %d inputs (gas_valid=%d, gas_resistance=%.0f ohm)",
             n_inputs, (field->status & BME68X_GASM_VALID_MSK) ? 1 : 0,
             field->gas_resistance);

    bsec_status = bsec_do_steps(bsec_inputs, n_inputs, bsec_outputs, &n_outputs);
    if (bsec_status != BSEC_OK) {
        ESP_LOGE(TAG, "bsec_do_steps failed (error %d)", bsec_status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "bsec_do_steps returned %d outputs", n_outputs);

    /* Step 7: Extract outputs and populate data structure */
    memset(data, 0, sizeof(bme68x_data_t));
    data->is_bme688 = g_bme68x.is_bme688;
    data->timestamp_ms = esp_timer_get_time() / 1000;

    /* Default values */
    data->temperature = field->temperature;
    data->humidity = field->humidity;
    data->pressure = field->pressure / 100.0f;  /* Pa to hPa */
    data->accuracy = 0;

    /* Parse BSEC outputs */
    for (uint8_t i = 0; i < n_outputs; i++) {
        switch (bsec_outputs[i].sensor_id) {
            case BSEC_OUTPUT_IAQ:
                data->iaq = (uint16_t)bsec_outputs[i].signal;
                data->accuracy = bsec_outputs[i].accuracy;
                ESP_LOGD(TAG, "Got IAQ output: %d (acc=%d)", data->iaq, data->accuracy);
                break;

            case BSEC_OUTPUT_STATIC_IAQ:
                data->iaq = (uint16_t)bsec_outputs[i].signal;
                data->accuracy = bsec_outputs[i].accuracy;
                ESP_LOGD(TAG, "Got STATIC_IAQ output: %d (acc=%d)", data->iaq, data->accuracy);
                break;

            case BSEC_OUTPUT_CO2_EQUIVALENT:
                data->co2_equiv = (uint16_t)bsec_outputs[i].signal;
                ESP_LOGD(TAG, "Got CO2 output: %d ppm", data->co2_equiv);
                break;

            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                data->breath_voc = (uint16_t)bsec_outputs[i].signal;
                ESP_LOGD(TAG, "Got VOC output: %d ppm", data->breath_voc);
                break;

            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                data->temperature = bsec_outputs[i].signal;  /* Use compensated value */
                break;

            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                data->humidity = bsec_outputs[i].signal;  /* Use compensated value */
                break;

            case BSEC_OUTPUT_RAW_PRESSURE:
                data->pressure = bsec_outputs[i].signal / 100.0f;  /* Pa to hPa */
                break;

            default:
                /* Ignore other outputs */
                break;
        }
    }

    /* Step 8: Check if state save needed */
    uint32_t age_sec = bme680_get_state_age();
    if (age_sec >= STATE_SAVE_INTERVAL_SEC) {
        ESP_LOGI(TAG, "Saving BSEC state (age: %lu hours)", age_sec / 3600);
        bme680_save_state();
    }

    ESP_LOGD(TAG, "Read: T=%.1f°C, H=%.1f%%, P=%.1fhPa, IAQ=%d, Acc=%d",
             data->temperature, data->humidity, data->pressure,
             data->iaq, data->accuracy);

    return ESP_OK;
}

/* ======================== Helper Functions ======================== */

const char* bme680_get_variant_name(void) {
    return g_bme68x.is_bme688 ? "BME688" : "BME680";
}

const char* bme680_get_bsec_version(void) {
    static char version_str[16];
    bsec_version_t version;
    bsec_get_version(&version);
    snprintf(version_str, sizeof(version_str), "%d.%d.%d.%d",
             version.major, version.minor, version.major_bugfix, version.minor_bugfix);
    return version_str;
}

uint8_t bme680_get_i2c_address(void) {
    return g_bme68x.initialized ? g_bme68x.i2c_address : 0;
}

bool bme680_is_bme688(void) {
    return g_bme68x.is_bme688;
}

uint32_t bme680_get_state_age(void) {
    if (g_bme68x.last_state_save_time_us == 0) {
        return 0;
    }
    uint64_t age_us = esp_timer_get_time() - g_bme68x.last_state_save_time_us;
    return (uint32_t)(age_us / 1000000);
}

uint64_t bme680_get_next_call_ms(void) {
    if (!g_bme68x.initialized) {
        return 0;
    }

    /* Return the cached next_call from last bsec_sensor_control() call
     * (which happens during bme680_read()) */
    return g_bme68x.bsec_settings.next_call / 1000000;
}

#else /* CONFIG_BME680_ENABLED */

/* Stub implementations when BME680 disabled */
#include <esp_err.h>

esp_err_t bme680_init(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t bme680_read(bme68x_data_t *data) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t bme680_save_state(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t bme680_reset_calibration(void) { return ESP_ERR_NOT_SUPPORTED; }
const char* bme680_get_variant_name(void) { return "disabled"; }
const char* bme680_get_bsec_version(void) { return "N/A"; }
uint8_t bme680_get_i2c_address(void) { return 0; }
bool bme680_is_bme688(void) { return false; }
uint32_t bme680_get_state_age(void) { return 0; }

#endif /* CONFIG_BME680_ENABLED */
