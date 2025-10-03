idf_component_register(SRCS 
    "src/LoRa.cpp"
    INCLUDE_DIRS "." "./src"
    REQUIRES arduino
             Adafruit_BusIO)