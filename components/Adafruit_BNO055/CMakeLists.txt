idf_component_register(SRCS 
    "Adafruit_BNO055.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino
             Adafruit_BusIO
             Adafruit_Sensor)