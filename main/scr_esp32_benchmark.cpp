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

#include <Adafruit_BMP5XX.h>
#include "Adafruit_Sensor.h"
//#include "Adafruit_BMP3XX.h"
//#include "Adafruit_ADXL375.h"
//#include "Adafruit_LSM6DSO32.h"
//#include "Adafruit_BNO055.h"
// #include "Adafruit_GPS.h"
// #include "LoRa.h"

// SRAD Imports
#include "SRAD_PHX.h"

// Sensor SPI init
#define SPI_SCLK_PIN 12
#define SPI_MISO_PIN 13
#define SPI_MOSI_PIN 11
#define SPI_MAX_TRSZ 4096

// SD+LoRa SPI Init
//#define VSPI_SCLK_PIN 18
//#define VSPI_MISO_PIN 17
//#define VSPI_MOSI_PIN 16
// #define VSPI_MAX_TRSZ 4092

// I^2C Init
//#define I2C_SDA 8
//#define I2C_SCL 9

// CS definitions
//#define BMP390_CS 10
#define BMP580_CS 10
//#define ADXL375_CS 5
//#define LSM6DSO32_CS 4
//#define LORA_CS 7

// Lo-Ra Control Pins
//#define LORA_RST 21
//#define LORA_IRQ 19
//#define LORA_FREQ 915E6

// Debug control definitions
#define DEBUG

// Chip Object Instantiation
//Adafruit_BMP3XX BMP;
Adafruit_BMP5xx BMP; // Create BMP5xx object
//ADXL(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, ADXL375_CS);
//Adafruit_LSM6DSO32 LSM;
//Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
//Adafruit_GPS GPS(&Wire);

void init_spi() {
    // set outputs/inputs for software spi
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_11, GPIO_MODE_INPUT);

    // start SPI bus
    SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1);
    BMP.begin(BMP580_CS, &SPI);
    //ADXL.begin();
    //LSM.begin_SPI(LSM6DSO32_CS, SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
}

// void init_I2C() {
//     Wire.begin(I2C_SDA, I2C_SCL);
// }

void BMP_task(void *pvParameter)
{
    while(1)
    {
        if(!BMP.performReading()) {
            taskYIELD();
        }
        float bmp_temp = BMP.temperature;
        float bmp_press = BMP.pressure;

        float bmp_alt = BMP.readAltitude(1013.25);

        TickType_t xUptime = xTaskGetTickCount();
        printf("Uptime [ms]: %lu\n", xUptime);

        #ifdef DEBUG
            printf("bmp_temp: %f\n", bmp_temp);
            printf("bmp_press: %f\n", bmp_press);
            printf("bmp_alt: %f\n\n", bmp_alt);
        #endif

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void app_main()
{
    // init Arduino Framework from ESP HAL
    initArduino();

    // init SPI buses
    init_spi();

    // init I^2C bus
    //init_I2C();

    // dump GPIO config
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    xTaskCreatePinnedToCore(BMP_task, "BMP_task", 5000, NULL, 1, NULL, 0);
}
