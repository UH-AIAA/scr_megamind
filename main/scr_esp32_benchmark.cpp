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
    BNO.begin();
}

void ADXL_task(void *pvParameter);
void BNO_task(void *pvParameter);
void LSM_task(void *pvParameter);

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
    
    xTaskCreatePinnedToCore(
        ADXL_task,
        "ADXL_task",
        5000,
        NULL,
        1,
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        BNO_task,
        "BNO_task",
        5000,
        NULL,
        1,
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        LSM_task,
        "LSM_task",
        5000,
        NULL,
        1,
        NULL,
        0
    );
}

void ADXL_task(void *pvParameter) {

    while(1) {
    //ADXL already initialized in previous code?
        //check ADXL connection and yield to other tasks if connection bad
        uint32_t startTime = millis();
        //Taking the time since activation of sensor
        TickType_t uptime = xTaskGetTickCount();

        sensors_event_t event;
        //stores data from the sensor into the sensor event variable "accelration"
        if(!ADXL.getEvent(&event)) {
            taskYIELD();
        }

        uint32_t endTime = millis();

        #ifdef DEBUG
            printf("ADXL Uptime: %lu [ms]\n",uptime);
            //Display the results (acceleration is measured in m/s^2)

            printf("X: %f [m/s^2]\n",event.acceleration.x);
            printf("Y: %f [m/s^2]\n",event.acceleration.y);
            printf("Z: %f [m/s^2]\n",event.acceleration.z);
            printf("Elapsed Time: %li\n\n", endTime - startTime);
        #endif

        //I believe the delay function in comment below is for arduino
        //delay(500);
        //delay 1 tick
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void BNO_task(void *pvParameter) {
    while (1) {
        // printf("BNO Task Called!\n");
        sensors_event_t orientationData, angVelocityData, magnetometerData, accelerometerData;

        if (!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
            taskYIELD();
        }
        if (!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
            taskYIELD();
        }
        if (!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER)) {
            taskYIELD();
        }
        if (!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER)) {
            taskYIELD();
        }
        
        uint32_t startTime = millis();
        TickType_t uptime = xTaskGetTickCount();
        
        imu::Quaternion quat = BNO.getQuat();

        uint32_t endTime = millis();

        #ifdef DEBUG
            printf("BNO Uptime: %lu\n", uptime);
            printf("Quaternion:\n");
            printf("W: %f\n", quat.w());
            printf("X: %f\n", quat.x());
            printf("Y: %f\n", quat.y());
            printf("Z: %f\n\n", quat.z());

            printf("Euler Orientation:\n");
            printf("X: %f\n", angVelocityData.gyro.x);
            printf("Y: %f\n", angVelocityData.gyro.y);
            printf("Z: %f\n", angVelocityData.gyro.z);
            printf("Elapsed Time: %li\n\n\n", endTime - startTime);
            // other data has been left out to avoid slowing down printing

        #endif

        vTaskDelay(pdMS_TO_TICKS(20 - (endTime - startTime)));
    }
}

void LSM_task(void *pvParameter) {
    while (1) {
        TickType_t uptime = xTaskGetTickCount();

        uint32_t startTime = millis();

        //Sensor events
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;

        //event to get data
        if(!LSM.getEvent(&accel, &gyro, &temp)) {
            taskYIELD();
        }

        //data printing 

        #ifdef DEBUG
            printf("LSM Ticktime: %lu [ms]\n", uptime);
            printf("X Acceleration: %f [m/s^2]\n", accel.acceleration.x);
            printf("Y Acceleration: %f [m/s^2]\n", accel.acceleration.y);
            printf("Z Acceleration: %f [m/s^2]\n", accel.acceleration.z);
            printf("X Gyro: %f [idk]\n",gyro.gyro.x);
            printf("Y Gyro: %f [idk]\n",gyro.gyro.y);
            printf("Z Gyro: %f [idk]\n",gyro.gyro.z);
            // printf("Temperature: %f [deg C]\n",temp);
            uint32_t endTime = millis();
            printf("Elapsed Time: %li\n\n", endTime - startTime);

        #endif
        
        //delay funct 
        //delay (500), 1 tick
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
