# main/CMakeLists.txt

idf_component_register(SRCS "main.c"
                            "veml7700_driver.c"
                            "opt4001_driver.c"
                            "als_driver.c"
                            "can_protocol.c"
                            "bme680_bsec.c"
                            "ota_handler.c"
                      INCLUDE_DIRS "."
                      REQUIRES driver esp_timer nvs_flash bsec app_update esp_partition)
