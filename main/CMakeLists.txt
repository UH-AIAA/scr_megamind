idf_component_register(SRCS "Ops.cpp" "State.cpp" "Sensors.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        arduino
                        LoRa
                        SRAD_PHX)

