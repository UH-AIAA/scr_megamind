idf_component_register(SRCS 
    "megamind.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino
            Adafruit_BusIO
            Adafruit_Sensor
            Adafruit_BMP5xx
            Adafruit_ADXL375
            Adafruit_LSM6DS
            Adafruit_BNO055
            Adafruit_GPS
            hawkeye)
