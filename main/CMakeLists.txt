# main/CMakeLists.txt

idf_component_register(SRCS "main.c"
                            "veml7700_driver.c"
                            "opt4001_driver.c"
                            "als_driver.c"
                            "can_protocol.c"
                      INCLUDE_DIRS ".")
