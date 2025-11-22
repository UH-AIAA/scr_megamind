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

#include "esp_system.h"
#include "driver/gpio.h"

// SPI imports, I^2C, and UART Imports
#include "SPI.h"
#include "Arduino.h"
#include "Adafruit_BMP5xx.h"
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
#define BMP581_CS 10
#define ADXL375_CS 5
#define LSM6DSO32_CS 4
#define LORA_CS 7

// Lo-Ra Control Pins
#define LORA_RST 21
#define LORA_IRQ 19
#define LORA_FREQ 915E6

// Debug control definitions
#define DEBUG

typedef struct{
    float bmp_temp;
    float bmp_press;
    float bmp_alt;
} OutputData_t;

typedef struct{
    OutputData_t *pOutputData;
    sensors_event_t *pEvent;
}TaskParams_t;

OutputData_t gOutputData; /// global Output object for sensors data
sensors_event_t gEvent; /// global output data for acceleration sensors

uint32_t upTime; /// The current time of task is running in the beginning

//Chip Object Instantiation
Adafruit_BMP5xx BMP;
Adafruit_ADXL375 ADXL(ADXL375_CS, &SPI);
// Adafruit_LSM6DSO32 LSM;
// Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
// Adafruit_GPS GPS(&Wire);


void init_spi() {
    SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1);

    // start SPI bus
    BMP.begin(BMP581_CS, &SPI);
    ADXL.begin();
    // LSM.begin_SPI(LSM6DSO32_CS, &SPI, 1000);
}

void init_I2C() {
    //Wire.begin(I2C_SDA, I2C_SCL);

    // // GPS Setup
    // GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    
    // BNO begin
    //BNO.begin();
}

void Core0_task(void *pvParameter);
void Core1_task(void *pvParameter);

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

    TaskParams_t *pParams = (TaskParams_t*)malloc(sizeof(TaskParams_t));
    pParams->pOutputData = &gOutputData;
    pParams->pEvent = &gEvent;

    xTaskCreatePinnedToCore(
        Core0_task,
        "LSM_task",
        5000,
        (void*)pParams,
        1,
        NULL,
        0
    );

    // xTaskCreatePinnedToCore(
    //     Core1_task,
    //     "BMP_task",
    //     5000,
    //     NULL,
    //     1,
    //     NULL,
    //     1
    // );
}

void Core0_task(void *pvParameter) {
    TaskParams_t *pTask = (TaskParams_t*)pvParameter;
    OutputData_t *pOutputData = pTask->pOutputData;
    sensors_event_t *pEvent = pTask->pEvent;

    while(1)
    {
        ///BMP function
        upTime = xTaskGetTickCount();
        if(!BMP.performReading()) {
            taskYIELD();
        }
        pOutputData->bmp_temp = BMP.temperature;
        pOutputData->bmp_press = BMP.pressure;
        pOutputData->bmp_alt = BMP.readAltitude(1013.25);

        #ifdef DEBUG
          printf("BMP Up Time: %ld [ms]\n",upTime);
          printf("bmp_temp: %f\n", pOutputData->bmp_temp);
          printf("bmp_press: %f\n", pOutputData->bmp_press);
          printf("bmp_alt: %f\n\n", pOutputData->bmp_alt);
        #endif


        ///ADXL function
        upTime = xTaskGetTickCount();
        if(!ADXL.getEvent(pEvent)) {
            taskYIELD();
        }
        #ifdef DEBUG
          printf("ADXL Up Time: %ld [ms]\n",upTime);
          printf("X: %f [m/s^2]\n",pEvent->acceleration.x);
          printf("Y: %f [m/s^2]\n",pEvent->acceleration.y);
          printf("Z: %f [m/s^2]\n\n",pEvent->acceleration.z);
        #endif


        // ///LSM function
        // upTime = xTaskGetTickCount();
        // if(!LSM.getEvent(pEvent)) {
        //     taskYIELD();
        // }
        // #ifdef DEBUG
        //     printf("LSM Up Time: %ld [ms]\n",upTime);
        //     printf("X Acceleration: %f [m/s^2]\n", pEvent->acceleration.x);
        //     printf("Y Acceleration: %f [m/s^2]\n", pEvent->acceleration.y);
        //     printf("Z Acceleration: %f [m/s^2]\n", pEvent->acceleration.z);
        //     printf("X Gyro: %f \n",gyro.gyro.x);
        //     printf("Y Gyro: %f \n",gyro.gyro.y);
        //     printf("Z Gyro: %f \n",gyro.gyro.z);
        //     // printf("Temperature: %f [deg C]\n",temp);
        //     uint32_t endTime = millis();
        //     printf("Elapsed Time: %li\n\n", endTime - startTime);
        // #endif

    }
}

void Core1_task(void *pvParameter) {

}