idf_component_register(SRCS 
    "src/Adafruit_GPS.cpp"
    "src/NMEA_build.cpp"
    "src/NMEA_data.cpp"
    "src/NMEA_parse.cpp"
    INCLUDE_DIRS "." "./src"
    REQUIRES arduino
             Adafruit_BusIO
             Adafruit_Sensor)