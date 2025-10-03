idf_component_register(SRCS 
    "SRAD_PHX_Ops.cpp"
    "SRAD_PHX_Sensors.cpp"
    "SRAD_PHX_State.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino
            Adafruit_BusIO
            Adafruit_Sensor
            Adafruit_BMP3XX
            Adafruit_ADXL375
            Adafruit_LSM6DS
            Adafruit_BNO055
            Adafruit_GPS)