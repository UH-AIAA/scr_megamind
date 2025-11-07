idf_component_register(SRCS "Ops.cpp" "State.cpp" "Main.cpp" 
                    INCLUDE_DIRS "."
                    REQUIRES 
                        arduino
                        LoRa
                        SRAD_PHX
                        Adafruit_BMP5xx
                        )

