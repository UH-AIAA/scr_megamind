idf_component_register(SRCS "Main.cpp" 
                    INCLUDE_DIRS "."
                    REQUIRES 
                        arduino
                        LoRa
                        SRAD_PHX
                        Adafruit_BMP5xx
                        )

