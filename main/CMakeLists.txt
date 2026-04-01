idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES 
                    arduino
                    LoRa
                    Adafruit_BusIO
                    Adafruit_Sensor
                    # Adafruit_BMP3XX
                    Adafruit_BMP5xx
                    Adafruit_ADXL375
                    Adafruit_LSM6DS
                    Adafruit_BNO055
                    Adafruit_GPS
                    scr_megamind)

