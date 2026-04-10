///////////////////////////////////////////////////////////////////////
/*                        SCR Megamind FSW                           */
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
#include "esp_timer.h"

// SPI imports, I^2C, and UART Imports
#include "SPI.h"
#include "SD.h"
#include "Arduino.h"
#include "Adafruit_BMP5xx.h"
#include "Adafruit_ADXL375.h"
#include "Adafruit_LSM6DSO32.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_GPS.h"
#include "LoRa.h"

// SRAD Imports
 #include "megamind.h"

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
#define SD_CS 20

// Lo-Ra Control Pins
#define LORA_RST 21
#define LORA_IRQ 19
#define LORA_FREQ 915E6

// Debug control definitions
#define DEBUG
#define MAX_SENSOR_QUEUE_SIZE (200)  // napkin math says this is abt 1s of data?

// Chip Object Instantiation
Adafruit_BMP5xx BMP;
Adafruit_ADXL375 ADXL(ADXL375_CS, &SPI);
Adafruit_LSM6DSO32 LSM;
Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS GPS(&Wire);
SPIClass SPI2(HSPI);
File sdData;

// SD Data Header
const char header[24] = "sensor_code,time,packet";
float ADXL_ACCEL_BIAS[3];
float LSM_ACCEL_BIAS[3];
float LSM_GYRO_BIAS[3];
float BMP_ALT_BIAS;

//SPI mutex
TaskHandle_t LORA_handle;
static SemaphoreHandle_t sensor_spi_mutex;
static SemaphoreHandle_t sd_lora_spi_mutex;

GDQ_t GDQ;

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

    LSM.setAccelDataRate(LSM6DS_RATE_104_HZ); //gives accelerometer data every 9.6ms
    // TODO: [DA] - configure LSM acceleration range based off of acceleration curve

    // TODO: [DA] - configure ADXL acceleration range based off of acceleration curve
    
    // // LORA setup (Thanh's work!)
    // LoRa.setSPI(SPI2);
    // LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    // LoRa.begin(LORA_FREQ);

    // SD setup
    if(!SPI2.begin(VSPI_SCLK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN, -1)) {
        while(1){
        printf("SPI2 failed\n");
        }
    }
    // if(!SD.begin(SD_CS, SPI2, 1E6))   // TODO: [NS] make this a #define
    // {
    //     printf("SD never began!\n");
    //     while (1) {
    //     }
    // }

    // NOTE: temporarily uncommented this for fried module
    // TODO: update file name with RTC input once configured
    // sdData = SD.open("/newName.csv", FILE_WRITE);
    // if(!sdData) {
    //     while(1) {
    //         printf("SD FILE FAILED\n");
    //     }
    // }
    // sdData.println(header);

    // LORA setup (Thanh's work!)
    // LoRa.setSPI(SPI2);
    // LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    // if(!LoRa.begin(LORA_FREQ))
    // {
    //     printf("LoRa.begin failed!\n");
    //     while (1) {};
    // }
}

void init_I2C() {
    Wire.begin(I2C_SDA, I2C_SCL);

    // GPS Setup
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);

    // BNO begin
    // TODO: [NS] - figure out how to set BNO to manual
    BNO.begin();
}

// init queues
void init_GDQ() {
    GDQ.SensorQueue = xQueueCreate(MAX_SENSOR_QUEUE_SIZE * 4, sizeof(GDQMessage_t));
    
    GDQ.LatestGPSMsg = {0};
    GDQ.LatestADXLMsg = {0};
    GDQ.LatestBNOMsg = {0};
    GDQ.LatestLSMMsg = {0};
    GDQ.LatestBMPMsg = {0};
}

// TODO: [NS/LF] figure out how to get these in another file to avoid polluting main
void ADXL_task(void *pvParameter);
void BNO_task(void *pvParameter);
void LSM_task(void *pvParameter);
void GPS_task(void *pvParameter);
void BMP_task(void *pvParameter);
void SD_task(void *pvParameter);
void LORA_task(void *pvParameter);

extern "C" void app_main()
{
    // init Arduino Framework from ESP HAL
    initArduino();

    // init SPI buses
    init_spi();

    // init I^2C bus
    init_I2C();

    // init GDQ
    init_GDQ();

    // dump GPIO config
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    // initialize Mutex
    sensor_spi_mutex = xSemaphoreCreateMutex();
    sd_lora_spi_mutex = xSemaphoreCreateMutex();  
    printf("Mutex addr: %p\n", sd_lora_spi_mutex);

    printf("Packet size %i\n", sizeof(LORAMessage_t));

    // TODO: [add calibration functions here for LSM] [NS]

    // TODO: [JF] call altimeter calibration function here!

    // sample task for your convenience
/*    xTaskCreate(
        MegaMind_LAUNCH,    // [in] function pointer
        "MegaMind_LAUNCH",  // [in] debug name, leave same as function name plz
        50000,              // [in] function stack frame size (bytes),
        NULL,               // [in] parameters to pass
        2,                  // [in] task prioirity
        NULL                // [in] task handle (leave null)
    );
*/
    // create tasks!
    xTaskCreatePinnedToCore(
        LORA_task,
        "LORA_task",
        10000,
        NULL,
        5,
        &LORA_handle,
        1
    );

    xTaskCreatePinnedToCore(
        GPS_task,
        "GPS Task",
        8000,
        NULL,
        3,
        NULL,
        1
    );

    // NOTE: temporarily commented out while waiting for new SD chip
    // xTaskCreatePinnedToCore(
    //     SD_task,
    //     "SD_task",
    //     8000,
    //     NULL,
    //     4,
    //     NULL,
    //     1
    // );

    xTaskCreatePinnedToCore(
        ADXL_task,
        "ADXL_task",
        5000,
        NULL,
        5,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        BNO_task,
        "BNO_task",
        5000,
        NULL,
        2,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        LSM_task,
        "LSM_task",
        5000,
        NULL,
        3,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        BMP_task,
        "BMP_task", 5000, NULL, 4,
        NULL,
        0
    );
}

void GPS_task(void *pvParameter) {
    GDQMessage_t currentMessage = {
        .sensor = SENSOR_GPS,
    };

    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t lastWake = xTaskGetTickCount();

    while(true){
        uint32_t startms = esp_timer_get_time();
        xTaskNotifyGive(LORA_handle);
        uint32_t timeout = startms + 200000;

        while (esp_timer_get_time() < timeout) {
            while (GPS.available() && esp_timer_get_time() < timeout) {
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

                        // add data to struct
                        currentMessage.GPSMessage.time = startms;
                        currentMessage.GPSMessage.satellites = GPS.satellites;
                        currentMessage.GPSMessage.latitude = GPS.latitude;
                        currentMessage.GPSMessage.longitude = GPS.longitude;
                        currentMessage.GPSMessage.lat = GPS.lat;
                        currentMessage.GPSMessage.lon = GPS.lon;
                        currentMessage.GPSMessage.altitude = GPS.altitude;

                        //write message to queue
                        GDQ.LatestGPSMsg = currentMessage.GPSMessage;
                        xQueueSendToBack(GDQ.SensorQueue, &currentMessage, 0);
                        // if we found data, go to end of function
                        // we don't want to print out the same data multiple times
                        goto end;
                    }
                }
            }
        }

        printf("GPS no fix!\n");

        end:
            uint32_t endms = esp_timer_get_time();
            uint32_t spentms = endms - startms;
            printf("Microseconds: %lu\n", endms);
            printf("Task took %lu ms to complete.\n\n", spentms);
            vTaskDelayUntil(&lastWake, period);
            
            // TODO: [NS] make LORA_task eligible to run
        // xTaskNotifyGive(LORA_handle);
    }
}


void ADXL_task(void *pvParameter) {
    // variable declarations
    GDQMessage_t currentMessage = {
        .sensor = SENSOR_ADXL,
    };
    uint32_t startTime, endTime;
    TickType_t uptime;
    static bool adxl_up;
    while(1) {
        // gather what time the function started
        startTime = esp_timer_get_time();
        uptime = xTaskGetTickCount();

        // attempt to take mutex, code inside blocked until mutex is released
        if(xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE){
            // if success,
            adxl_up = ReadADXL(&ADXL, &currentMessage, LSM_ACCEL_BIAS);
            // gives the mutex back
            xSemaphoreGive(sensor_spi_mutex);
        }

        if (adxl_up) {
            currentMessage.time = startTime;
            // write data to queue
            xQueueSendToBack(GDQ.SensorQueue, &currentMessage, 0);
            GDQ.LatestADXLMsg = currentMessage.ADXLMessage;
            

            endTime = esp_timer_get_time();
    
            #ifdef DEBUG
                printf("ADXL Uptime: %lu [ms]\n",uptime);
                //Display the results (acceleration is measured in m/s^2)

                printf("X: %f [m/s^2]\n",currentMessage.ADXLMessage.acceleration[0]);
                printf("Y: %f [m/s^2]\n",currentMessage.ADXLMessage.acceleration[1]);
                printf("Z: %f [m/s^2]\n",currentMessage.ADXLMessage.acceleration[2]);
                printf("Elapsed Time: %li\n\n", endTime - startTime);
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// TODO: [NS/JL] figure out exactly raw data we have access
//               to and what it means
void BNO_task(void *pvParameter) {
    // variable declaration
    uint32_t startTime, endTime;
    TickType_t uptime;
    GDQMessage_t currentMessage = {
        .sensor = SENSOR_BNO,
    };

    while (1) {
        // TODO: [NS/JL] assess if we actually need to gather all this data from the sensor
        // if we can't get BNO data, end task early and schedule again after 1 tick

        // gather function start time
        startTime = esp_timer_get_time();
        uptime = xTaskGetTickCount();

        if(ReadBNO(&BNO, &currentMessage))
        {
            currentMessage.time = startTime;

            //write message to queue
            xQueueSendToBack(GDQ.SensorQueue, &currentMessage, 0);
            GDQ.LatestBNOMsg = currentMessage.BNOMessage;
        }
        
        endTime = esp_timer_get_time();

        #ifdef DEBUG
            printf("BNO Uptime: %lu\n", uptime);
            printf("Quaternion:\n");
            printf("W: %f\n", currentMessage.BNOMessage.quaternion[0]);
            printf("X: %f\n", currentMessage.BNOMessage.quaternion[1]);
            printf("Y: %f\n", currentMessage.BNOMessage.quaternion[2]);
            printf("Z: %f\n\n", currentMessage.BNOMessage.quaternion[3]);

            printf("Euler Orientation:\n");
            printf("X: %f\n", currentMessage.BNOMessage.euler_orientation[0]);
            printf("Y: %f\n", currentMessage.BNOMessage.euler_orientation[1]);
            printf("Z: %f\n", currentMessage.BNOMessage.euler_orientation[2]);
            printf("Elapsed Time: %li\n\n\n", endTime - startTime);
            // other data has been left out to avoid slowing down printing

        #endif

        // vTaskDelay(pdMS_TO_TICKS(20 - (endTime - startTime)));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void LSM_task(void *pvParameter) {

    GDQMessage_t currentMessage = {
        .sensor = SENSOR_LSM,
    };

    TickType_t uptime;
    uint32_t startTime;
    uint32_t endTime;

    while (1) {
        uptime = xTaskGetTickCount(); 
        startTime = esp_timer_get_time(); 

        // attempts to retrieve mutex
        if(xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE)
        {
            // remove comment marks to include temperate
            //currentMessage.temp = temp;
            if(ReadLSM(&LSM, &currentMessage, LSM_ACCEL_BIAS, LSM_GYRO_BIAS)) {
                currentMessage.time = startTime;
                // writes data to queue
                if(xQueueSendToBack(GDQ.SensorQueue, &currentMessage, 0) != pdTRUE)
                {
                    // NOTE: temporarily commented out for work on unstable branch
                    // printf("LSM Lost Packet!\n");
                }
                GDQ.LatestLSMMsg = currentMessage.LSMMessage;
            }
            
            // give back mutex
            xSemaphoreGive(sensor_spi_mutex);

            endTime = esp_timer_get_time();

            //data printing
            #ifdef DEBUG
                printf("LSM Ticktime: %lu [ms]\n", uptime);
                printf("X Acceleration: %f [m/s^2]\n", currentMessage.LSMMessage.acceleration[0]);
                printf("Y Acceleration: %f [m/s^2]\n", currentMessage.LSMMessage.acceleration[1]);
                printf("Z Acceleration: %f [m/s^2]\n", currentMessage.LSMMessage.acceleration[2]);
                printf("X Gyro: %f [idk]\n",currentMessage.LSMMessage.gyro[0]);
                printf("Y Gyro: %f [idk]\n",currentMessage.LSMMessage.gyro[1]);
                printf("Z Gyro: %f [idk]\n",currentMessage.LSMMessage.gyro[2]);
                // printf("Temperature: %f [deg C]\n",temp);
                printf("Elapsed Time: %li\n\n", endTime - startTime);

            #endif

        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void BMP_task(void *pvParameter) {
    static bool bmp_up;
    static float bmp_temp, bmp_press, bmp_alt;
    static TickType_t upTime;
    static uint32_t startTime, endTime;
    //initialize struct
    GDQMessage_t currentMessage = {
        .sensor = SENSOR_BMP,
    };
    while(1) {
        // attempts to retrieve mutex
        if(xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE){

            // time system
            startTime = esp_timer_get_time();
            upTime = xTaskGetTickCount();
            bmp_up = ReadBMP(&BMP, &currentMessage, &BMP_ALT_BIAS);
            xSemaphoreGive(sensor_spi_mutex);//gives back mutex
            endTime = esp_timer_get_time();

            if(bmp_up) {
                // TODO: [NS] add calibration

                // adds data to struct
                currentMessage.BMPMessage.time = startTime;
                // writes data to queue
                xQueueSendToBack(GDQ.SensorQueue, &currentMessage, 0);
                GDQ.LatestBMPMsg = currentMessage.BMPMessage;

                #ifdef DEBUG
                    printf("BMP reporting OK!\n");
                    printf("BMP Temp: %f\n", currentMessage.BMPMessage.temp);
                    printf("BMP Press: %f\n", currentMessage.BMPMessage.pressure);
                    printf("BMP Alt: %f\n", currentMessage.BMPMessage.altitude);
                    printf("Uptime [ms/ticks]: %lu\n\n", upTime);
                    printf("Elapsed Time: %li\n\n\n", endTime - startTime);
                #endif
            } else {
                #ifdef DEBUG
                    printf("BMP reporting NOT OK!\n\n");
                #endif
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void SD_task(void *pvParameter)
{
    char msgBuf[4096];
    size_t index = 0;
    size_t remaining;
    static int n;
    static uint32_t flushCounter = 0;
    

    GDQMessage_t currentMessage;

    // TODO: [NS] make these #defines

    while(1)
    {
        // TODO: [NS] SD SPI Mutex
        // printf("started sd...\n");

        // block task until queue has data
        xQueueReceive(GDQ.SensorQueue, &currentMessage, portMAX_DELAY);
        uint32_t startTime = esp_timer_get_time();
        TickType_t uptime = xTaskGetTickCount();

        remaining = sizeof(msgBuf) - index;

        switch(currentMessage.sensor) {
            case SENSOR_GPS:
                n += snprintf(msgBuf + index, remaining,
                     "%i,%llu,%i,%f,%f,%c,%c,%f\n",
                     SENSOR_GPS,
                     currentMessage.GPSMessage.time,
                     currentMessage.GPSMessage.satellites,
                     currentMessage.GPSMessage.latitude, currentMessage.GPSMessage.longitude,
                     currentMessage.GPSMessage.lat, currentMessage.GPSMessage.lon,
                     currentMessage.GPSMessage.altitude
                );
                break;
            case SENSOR_ADXL:
                n += snprintf(msgBuf + index, remaining,
                     "%i,%llu,%f,%f,%f\n",
                     SENSOR_ADXL,
                     currentMessage.ADXLMessage.time,
                     currentMessage.ADXLMessage.acceleration[0], currentMessage.ADXLMessage.acceleration[1], currentMessage.ADXLMessage.acceleration[2]
                );
                break;
            case SENSOR_BNO:
                n += snprintf(msgBuf + index, remaining,
                     "%i,%llu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                     SENSOR_BNO,
                     currentMessage.BNOMessage.uptime,
                     currentMessage.BNOMessage.quaternion[0], currentMessage.BNOMessage.quaternion[1], currentMessage.BNOMessage.quaternion[2], currentMessage.BNOMessage.quaternion[3],
                     currentMessage.BNOMessage.acceleration[0], currentMessage.BNOMessage.acceleration[1], currentMessage.BNOMessage.acceleration[2],
                     currentMessage.BNOMessage.euler_orientation[0], currentMessage.BNOMessage.euler_orientation[1], currentMessage.BNOMessage.euler_orientation[2],
                     currentMessage.BNOMessage.magnetometer[0], currentMessage.BNOMessage.magnetometer[1], currentMessage.BNOMessage.magnetometer[2]
                );
                break;

            case SENSOR_LSM:
                n += snprintf(msgBuf + index, remaining,
                     "%i,%llu,%f,%f,%f,%f,%f,%f\n",
                     SENSOR_LSM,
                     currentMessage.LSMMessage.time,
                     currentMessage.LSMMessage.acceleration[0], currentMessage.LSMMessage.acceleration[1], currentMessage.LSMMessage.acceleration[2],
                     currentMessage.LSMMessage.gyro[0], currentMessage.LSMMessage.gyro[1], currentMessage.LSMMessage.gyro[2]
                );
                break;
            case SENSOR_BMP:
                n += snprintf(msgBuf + index, remaining,
                     "%i,%llu,%f,%f,%f\n",
                     SENSOR_BMP,
                     currentMessage.BMPMessage.time,
                     currentMessage.BMPMessage.temp,
                     currentMessage.BMPMessage.pressure,
                     currentMessage.BMPMessage.altitude
                );
                break;
        }

        // if buffer full, write out
        if(n >= remaining) {
            xSemaphoreTake(sd_lora_spi_mutex, portMAX_DELAY);

            sdData.print(msgBuf);
            flushCounter += index;
            if(flushCounter >= 32E3) {
                sdData.flush();
                flushCounter = 0;
            }
            #ifdef DEBUG
                uint32_t endTime = esp_timer_get_time();
                printf("Uptime: %lu\n", uptime);
                printf("ELAPSED TIME: %lu\n", endTime - startTime);
                printf("Used Bytes: %lu\n", (unsigned long)index);
                printf("Queue Length: %lu\n\n", (unsigned long)uxQueueMessagesWaiting(GDQ.SensorQueue));
            #endif

            xSemaphoreGive(sd_lora_spi_mutex);

            // reset buffer
            index = 0;
            n = 0;
            memset(msgBuf, 0, sizeof(msgBuf));
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            index += n;
        }
    }
}

void LORA_task(void *pvParameter)
{
    static TickType_t uptime;
    static uint32_t startTime, endTime;
    LORAMessage_t currentMessage;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uptime = xTaskGetTickCount();
        startTime = esp_timer_get_time();

        // compress to current message
        currentMessage.ADXL_time = GDQ.LatestADXLMsg.time;
        currentMessage.ADXL_accel[0] = (int16_t)(GDQ.LatestADXLMsg.acceleration[0] * 1000);
        currentMessage.ADXL_accel[1] = (int16_t)(GDQ.LatestADXLMsg.acceleration[1] * 1000);
        currentMessage.ADXL_accel[2] = (int16_t)(GDQ.LatestADXLMsg.acceleration[2] * 1000);

        currentMessage.BNO_time = GDQ.LatestBNOMsg.uptime;
        currentMessage.BNO_quat[0] = (int16_t)(GDQ.LatestBNOMsg.quaternion[0] * 1000);
        currentMessage.BNO_quat[1] = (int16_t)(GDQ.LatestBNOMsg.quaternion[1] * 1000);
        currentMessage.BNO_quat[2] = (int16_t)(GDQ.LatestBNOMsg.quaternion[2] * 1000);
        currentMessage.BNO_quat[3] = (int16_t)(GDQ.LatestBNOMsg.quaternion[3] * 1000);
        
        currentMessage.LSM_time = GDQ.LatestLSMMsg.time;
        // printf("lsm time: ")
        currentMessage.LSM_accel[0] = (int16_t)(GDQ.LatestLSMMsg.acceleration[0] * 1000);
        currentMessage.LSM_accel[1] = (int16_t)(GDQ.LatestLSMMsg.acceleration[1] * 1000);
        currentMessage.LSM_accel[2] = (int16_t)(GDQ.LatestLSMMsg.acceleration[2] * 1000);

        // TODO: [NS] Fix this by adding the rest of the data

        currentMessage.BMP_time = GDQ.LatestBMPMsg.time;
        currentMessage.BMP_temp = (int16_t)(GDQ.LatestBMPMsg.temp);
        currentMessage.BMP_pressure = (int16_t)(GDQ.LatestBMPMsg.pressure);
        currentMessage.BMP_altitude = (int16_t)(GDQ.LatestBMPMsg.altitude * 1000);

        currentMessage.GPS_time = GDQ.LatestGPSMsg.time;
        currentMessage.GPS_sat = (uint8_t)(GDQ.LatestGPSMsg.satellites);
        currentMessage.GPS_lat = (int16_t)(GDQ.LatestGPSMsg.latitude);
        currentMessage.GPS_lon = (int16_t)(GDQ.LatestGPSMsg.longitude);
        currentMessage.GPS_lat_dir = GDQ.LatestGPSMsg.lat;
        currentMessage.GPS_lon_dir = GDQ.LatestGPSMsg.lon;
        currentMessage.GPS_alt = GDQ.LatestGPSMsg.altitude;

        // flight state/apogee TODO: [NS] update with state machine
        currentMessage.apogeeEstimate = 0;
        currentMessage.flightState = 0;

        if(xSemaphoreTake(sd_lora_spi_mutex, portMAX_DELAY) == pdTRUE)
        {
            // write packet!
            LoRa.beginPacket();
            LoRa.write((uint8_t*)&currentMessage, sizeof(LORAMessage_t));
            LoRa.endPacket(true);

            xSemaphoreGive(sd_lora_spi_mutex);

            #ifdef DEBUG
                endTime = esp_timer_get_time();

                printf("Time Elapsed LoRa: %lu\n", endTime - startTime);
            #endif
        }   
    }
}