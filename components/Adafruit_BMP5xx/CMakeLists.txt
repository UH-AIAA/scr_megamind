idf_component_register(SRCS 
    "Adafruit_BMP5xx.cpp"
    "bmp5.c"
    INCLUDE_DIRS "."
    REQUIRES Adafruit_Sensor Adafruit_BusIO
    )