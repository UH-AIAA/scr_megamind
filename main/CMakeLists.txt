idf_component_register(SRCS "scr_esp32_benchmark.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        arduino
                        LoRa
                        SRAD_PHX)

