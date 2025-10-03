idf_component_register(SRCS 
    "Adafruit_LSM6DSO32.cpp"
    "Adafruit_LSM6DS.cpp"
    "Adafruit_LSM6DSOX.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        Adafruit_BusIO
        Adafruit_Sensor
    )
