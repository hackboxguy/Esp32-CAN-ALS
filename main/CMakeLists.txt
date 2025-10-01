# main/CMakeLists.txt

idf_component_register(SRCS "main.c"
                            "veml7700_driver.c"
                            "can_protocol.c"
                      INCLUDE_DIRS ".")
