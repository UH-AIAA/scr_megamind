#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <utility/imumaths.h>

#include "esp_system.h"
#include "driver/gpio.h"

// SPI imports, I^2C, and UART Imports
#include "SPI.h"
#include "Arduino.h"
#include "Adafruit_LSM6DSO32.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_GPS.h"
#include "LoRa.h"

#include "BMP581.h"
#include "ADXL375.h"
#include "Pins.h"

// SRAD Imports
#include "SRAD_PHX.h"
// Debug control definitions
#define DEBUG

// Chip Object Instantiation
Adafruit_LSM6DSO32 LSM;
Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS GPS(&Wire);

void init_spi() {
    // set outputs/inputs for software spi
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_11, GPIO_MODE_INPUT);

    // start SPI bus
    SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1);
    LSM.begin_SPI(LSM6DSO32_CS, SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
}

void init_I2C() {
    Wire.begin(I2C_SDA, I2C_SCL);
    BNO.begin();
}

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

    xTaskCreatePinnedToCore(
        BMP_task, 
        "BMP_task", 
        5000, 
        NULL, 
        1, 
        NULL, 
        0);
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
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
