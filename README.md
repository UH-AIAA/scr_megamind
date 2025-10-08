Test BMP581 sensor:
1) Setup with ESP32S3-Wroom-1 dev module. (See the code for respective pins connection)
2) No begin_SPI() function comparing to BMP390 library so it has to be separately initialized into 2 funcitons
3) Reading temperature, pressure, altitude and print out time with those data at a rate of 1 tick (1 tick/1000 Hz = 1ms)
4) Running on core 0 by RTOS

Reworked library:
1) Header files and cpp files are moved outside of src folder to main folder to hide abstraction and reduce error
2) Created CMakeLists.txt file, allowing ESP-IDF to compile correctly using BMP581 library
3) Making the library local to ensure consistency process of development
