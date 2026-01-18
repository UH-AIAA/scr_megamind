idf_component_register(SRCS "prototype_main/src/prototype_main.cpp" "prototype_main/src/state_logic.cpp" "prototype_main/src/calibration_logic.cpp" 
                    INCLUDE_DIRS "." 
                    REQUIRES 
                        arduino
                        LoRa
                        SRAD_PHX
                        Adafruit_BMP5xx
                        )

