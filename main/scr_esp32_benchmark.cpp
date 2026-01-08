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
#include "esp_task_wdt.h"

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
Adafruit_BMP5xx BMP;
Adafruit_ADXL375 ADXL(ADXL375_CS, &SPI);
Adafruit_LSM6DSO32 LSM;
Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS GPS(&Wire);
// SPIClass SPI2(HSPI);
File sdData;

//SPI mutex
static SemaphoreHandle_t mutex;



void init_spi() {
    // use Arduino SPI (for now...)
    printf("made it into init_spi\n");
    bool spiStatus = SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1);
    
    // start SPI sensors on bus
    BMP.begin(BMP390_CS, &SPI);
    ADXL.begin();
    LSM.begin_SPI(LSM6DSO32_CS, &SPI);

    // TODO: [NS] add SD/Lora SPI

    // sensor setup
    BMP.setPressureOversampling(BMP5XX_OVERSAMPLING_1X);
    BMP.setOutputDataRate(BMP5XX_ODR_50_HZ);
    BMP.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    BMP.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
}

void init_I2C() {
    Wire.begin(I2C_SDA, I2C_SCL);

    // GPS Setup
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    
    // BNO begin
    BNO.begin();
}

// TODO: [NS/LF] figure out how to get these in another file to avoid polluting main
void ADXL_task(void *pvParameter);
void BNO_task(void *pvParameter);
void LSM_task(void *pvParameter);
void GPS_task(void *pvParameter);
void BMP_task(void *pvParameter);

void MegaMind_LAUNCH(void *pvParameter);

extern "C" void app_main()
{
    // init Arduino Framework from ESP HAL
    initArduino();

    // init SPI buses
    init_spi();

    // init I^2C bus
    vTaskDelay(pdMS_TO_TICKS(1));
    init_I2C();

    // dump GPIO config
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    //initiate Mutex, I assume this is the right place to do it
    mutex = xSemaphoreCreateMutex();

    // create launch task
    xTaskCreate(
        MegaMind_LAUNCH,    // [in] function pointer
        "MegaMind_LAUNCH",  // [in] debug name, leave same as function name plz
        50000,              // [in] function stack frame size (bytes),
        NULL,               // [in] parameters to pass
        2,                  // [in] task prioirity
        NULL                // [in] task handle (leave null)
    );
}

// main RTOS entry point, trying to start arduino spi in app_main
// causes RTOS infrastructure problems bc ESP + Arduino is shitty
void MegaMind_LAUNCH(void *pvParameter) {
    // create tasks!
    xTaskCreate(
        GPS_task,
        "GPS Task",
        5000,
        NULL,
        3,
        NULL
    );
    
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
        0
    );

    // delete task after it starts everything
    vTaskDelete(NULL);
}

void GPS_task(void *pvParameter) {

    while(true){
        uint32_t startms = millis();
        uint32_t timeout = startms + 200;

        while (millis() < timeout) {
            while (GPS.available()) {
                GPS.read();
                //choke point is from GPS.read();
                //can only read 1 byte at a time

                if (GPS.newNMEAreceived()) {
                    if (!GPS.parse(GPS.lastNMEA())) {
                        continue;
                    }

                    if (GPS.fix && GPS.satellites > 0) {
                        printf("Satellites: %i\n", GPS.satellites);
                        printf("Latitude: %f, %c\n", GPS.latitude,GPS.lat);
                        printf("Longitude: %f, %c\n", GPS.longitude, GPS.lon);
                        printf("Altitude: %f [meters]\n", GPS.altitude);
                        
                        //Collects speed over the ground, not sure how useful it'll be
                        //printf("Speed (knots): %f\n" GPS.speed);

                        // if we found data, go to end of function
                        // we don't want to print out the same data multiple times
                        goto end;
                    }
                }
            }
        }

        end:
            uint32_t endms = millis();
            uint32_t spentms = endms - startms;
            printf("Millis: %lu\n", endms);
            printf("Task took %lu ms to complete.\n\n", spentms);
            uint32_t delayms = std::min(200 - spentms, static_cast<uint32_t>(10));
            vTaskDelay(delayms);
    }
}

void ADXL_task(void *pvParameter) {

    while(1) {

    //take mutex
    //will only execute if mutux in open
    if(xSemaphoreTake(mutex, 0) == pdTRUE){

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

        //gives the mutex back
        xSemaphoreGive(mutex);
    }
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

        //checks for mutex
        if(xSemaphoreTake(mutex, 0) == pdTRUE){

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

         xSemaphoreGive(mutex);//gives back mutex

    }
    }
}

void BMP_task(void *pvParameter) {
    static bool bmp_up;
    static float bmp_temp, bmp_press, bmp_alt;
    static TickType_t upTime;
    static uint32_t startTime, endTime;
    while(1) {

        //checks for mutex
        if(xSemaphoreTake(mutex, 0) == pdTRUE){

        // time systea
        startTime = millis();
        upTime = xTaskGetTickCount();
        bmp_up = BMP.performReading();

        if(bmp_up) {
            bmp_temp = BMP.temperature;
            bmp_press = BMP.pressure;
            bmp_alt = BMP.readAltitude(1013.25f);

            // TODO: [NS] add calibration

            #ifdef DEBUG
                printf("BMP reporting OK!\n");
                printf("BMP Temp: %f\n", bmp_temp);
                printf("BMP Press: %f\n", bmp_press);
                printf("BMP Alt: %f\n", bmp_alt);
                printf("Uptime [ms/ticks]: %lu\n\n", upTime);
                endTime = millis();
                printf("Elapsed Time: %li\n\n\n", endTime - startTime);
            #endif
            vTaskDelay(pdMS_TO_TICKS(10));  // TODO: [NS] optimize timing
        } else {
            #ifdef DEBUG
                printf("BMP reporting NOT OK!\n\n");
            #endif
        }

    // cleanup:
    //     vTaskDelay(pdMS_TO_TICKS(10));
    //     taskYIELD();
        xSemaphoreGive(mutex);//gives back mutex

        }
    }
}
