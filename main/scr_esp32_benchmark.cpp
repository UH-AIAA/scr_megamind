///////////////////////////////////////////////////////////////////////
/*                   SCR ESP-32 Brute Sensor Test                    */
///////////////////////////////////////////////////////////////////////
/*                           N. Samuell                              */
/*                      FreeRTOS/ESP-IDF test                        */
/*                          MIT License                              */
///////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <utility/imumaths.h>

#include "esp_system.h"
#include "driver/gpio.h"

// SPI imports, I^2C, and UART Imports
#include "SPI.h"
#include "Arduino.h"
#include "Adafruit_BMP3XX.h"
#include "Adafruit_ADXL375.h"
#include "Adafruit_LSM6DSO32.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_GPS.h"
#include "LoRa.h"

// SRAD Imports
#include "SRAD_PHX.h"

// Sensor SPI init
#define SPI_SCLK_PIN 12
#define SPI_MISO_PIN 13
#define SPI_MOSI_PIN 11
#define SPI_MAX_TRSZ 4096

// SD+LoRa SPI Init
#define VSPI_SCLK_PIN 18
#define VSPI_MISO_PIN 17
#define VSPI_MOSI_PIN 16
// #define VSPI_MAX_TRSZ 4092

// I^2C Init
#define I2C_SDA 8
#define I2C_SCL 9

// CS definitions
#define BMP390_CS 10
#define ADXL375_CS 5
#define LSM6DSO32_CS 4
#define LORA_CS 7

// Lo-Ra Control Pins
#define LORA_RST 21
#define LORA_IRQ 19
#define LORA_FREQ 915E6

// Debug control definitions
#define DEBUG

// Chip Object Instantiation
Adafruit_BMP3XX BMP;
Adafruit_ADXL375 ADXL(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, ADXL375_CS);
Adafruit_LSM6DSO32 LSM;
Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS GPS(&Wire);

//System Data
struct SystemData {
    int sensor_status[10];
    float latitude;
    float longitude;
    // ... add any other data you need to store ...
};

SystemData myData;

void init_spi() {
    // set outputs/inputs for software spi
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_11, GPIO_MODE_INPUT);

    // start SPI bus
    BMP.begin_SPI(BMP390_CS, SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    ADXL.begin();
    LSM.begin_SPI(LSM6DSO32_CS, SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
}

void init_I2C() {
    Wire.begin(I2C_SDA, I2C_SCL);
}

void GPS_task(void *pvParameter){

    TickType_t start_time,end_time,elapsed_time;

        uint32_t startms = millis();
        uint32_t timeout = startms + 1000;

        start_time = xTaskGetTickCount();

        while (millis() < timeout) {
            while (GPS.available()) {
                GPS.read();
                //choke point is from GPS.read();
                //can only read 1 byte at a time

                if (GPS.newNMEAreceived()) {
                    // Serial.println(GPS.lastNMEA());
                    if (!GPS.parse(GPS.lastNMEA())) {
                        continue;
                    }

                    if (GPS.fix && GPS.satellites > 0) {
                        // Serial.print("Satellites: ");
                        // Serial.println(GPS.satellites);
                        //data.sensor_status[4] = 0;
                        myData.sensor_status[4] = 0; // 0 = OK
                        myData.latitude = GPS.latitude;
                        
                        printf("Latitude: %f\n", GPS.latitude);

                        end_time = xTaskGetTickCount();
                        elapsed_time = end_time - start_time;
                        printf("Task took %lu ticks to complete.\n\n", (unsigned long)elapsed_time);

                        break;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        vTaskDelete(NULL);
    
}




extern "C" void app_main()
{
    // init Arduino Framework from ESP HAL
    initArduino();

    // init SPI buses
    init_spi();

    // init I^2C bus
    init_I2C();

    // dump GPIO config
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    xTaskCreate(
        GPS_task,     // Arg 1: The function to run
        "GPS Task",   // Arg 2: A name for debugging
        5000,         // Arg 3: Stack size (memory for the task)
        NULL,         // Arg 4: The parameter to pass to the task
        3,            // Arg 5: The task's priority
        NULL          // Arg 6: The task handle (NULL is fine)
    );


    
}
