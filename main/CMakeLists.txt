idf_component_register(SRCS "Ops.cpp" "State.cpp" "Main.cpp" "BMP581/BMP581.cpp" "ADXL375/ADXL375.cpp"
                    INCLUDE_DIRS "." "BMP581" "ADXL375"
                    REQUIRES 
                        arduino
                        LoRa
                        SRAD_PHX
                        Adafruit_BMP5xx
                        )

