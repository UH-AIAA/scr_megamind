idf_component_register(SRCS 
    "src/RTC_DS1307.cpp"
    "src/RTClib.cpp"
    INCLUDE_DIRS "." "./src"
    REQUIRES arduino
             Adafruit_BusIO)