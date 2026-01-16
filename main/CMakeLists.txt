idf_component_register(SRCS "src/prototype_main.cpp" 
                    INCLUDE_DIRS "." "include"
                    REQUIRES 
                        arduino
                        LoRa
                        SRAD_PHX
                        Adafruit_BMP5xx
                        )

