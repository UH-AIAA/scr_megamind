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
#define MAX_SENSOR_QUEUE_SIZE (50)  // napkin math says this is abt 1s of data?

// Chip Object Instantiation
Adafruit_BMP5xx BMP;
Adafruit_ADXL375 ADXL(ADXL375_CS, &SPI);
Adafruit_LSM6DSO32 LSM;
Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS GPS(&Wire);
// SPIClass SPI2(HSPI);
File sdData;

//SPI mutex
static SemaphoreHandle_t sensor_spi_mutex;

// SENSOR DATA TYPES
typedef struct ADXLMessage {
    uint32_t time;          // ms since start for now (what we're already doing), maybe move to unixtime/RTC later
    float acceleration[3];  // acceleration data (x, y, z);
} ADXLMessage_t;

typedef struct BNOMessage {
    uint32_t uptime;              //uptime from BNO snsor
    float quaternion[4];          //W,X,Y,Z data
    float euler_orientation[3];   //x,y,z angular acceleration
    float magnetometer[3];        //x,y,z, of SOMETHING, not sure yet
    float acceleration[3];        //x,y,z linear acceleration
} BNOMessage_t;

typedef struct GPSMessage {
    uint32_t time;
    int satellites;               // not sure if this is necessary but I'll include it
    float longitude, latitude;    // numerical coordinates for the longitude and latitude in degress/minutes
    char lon,lat;                 // stores the cardinal direction (E/W for lon and N/S for lat)
    float altitude;               // gps altitude reading
}GPSMessage_t;

typedef struct LSMMessage {
    uint32_t time;
    float acceleration[3];         // stores the X,Y,Z acceleration
    float gyro[3];                 // stores X,Y,Z gyro orientation
    //float temp;                  // this was commented off but I'll keep it here in case we use it
}LSMMessage_t;

typedef struct BMPMessage {
    uint32_t time;
    float temp, pressure, alititude;    // temperature in celcius, pressure in pascals, altitude from sea level
}BMPMessage_t;


// SENSOR DATA STORAGE QUEUES
// GDQ == Global Data Queue
typedef struct GDQ {
    QueueHandle_t ADXL;
    QueueHandle_t BNO;
    // add more sensors here
    // QueueHandle_t GPS;
    QueueHandle_t LSM;
    QueueHandle_t BMP;
    GPSMessage_t GPSMsg;    // added to make globally available
} GDQ_t;


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

// init queues
void init_GDQ() {
    GDQ.ADXL = xQueueCreate(MAX_SENSOR_QUEUE_SIZE, sizeof(ADXLMessage_t));
    // do for other sensors here
    GDQ.BNO = xQueueCreate(MAX_SENSOR_QUEUE_SIZE, sizeof(BNOMessage_t));
    // GDQ.GPS = xQueueCreate(MAX_SENSOR_QUEUE_SIZE, sizeof(GPSMessage_t));
    GDQ.LSM = xQueueCreate(MAX_SENSOR_QUEUE_SIZE, sizeof(LSMMessage_t));
    GDQ.BMP = xQueueCreate(MAX_SENSOR_QUEUE_SIZE, sizeof(BMPMessage_t));
}

// TODO: [NS/LF] figure out how to get these in another file to avoid polluting main
void ADXL_task(void *pvParameter);
void BNO_task(void *pvParameter);
void LSM_task(void *pvParameter);
void GPS_task(void *pvParameter);
void BMP_task(void *pvParameter);

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
        GPS_task,
        "GPS Task",
        5000,
        NULL,
        3,
        NULL,
        1
    );
    
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

                        // add data to struct
                        GDQ.GPSMsg.satellites = GPS.satellites;
                        GDQ.GPSMsg.latitude = GPS.latitude;
                        GDQ.GPSMsg.longitude = GPS.longitude;
                        GDQ.GPSMsg.lat = GPS.lat;
                        GDQ.GPSMsg.lon = GPS.lon;
                        GDQ.GPSMsg.altitude = GPS.altitude;

                        //write message to queue

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
    // variable declarations
    ADXLMessage_t currentMessage = {0};
    sensors_event_t event;
    uint32_t startTime, endTime;
    TickType_t uptime;
    while(1) {
        // attempt to take mutex, code inside blocked until mutex is released
        if(xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE){

            // gather what time the function started
            startTime = millis();
            uptime = xTaskGetTickCount();
    
            // if read operation fails, start task over:
            if(!ADXL.getEvent(&event)) {
                xSemaphoreGive(sensor_spi_mutex);
                vTaskDelay(1);  // ends task, not eligible to be ran for another tick
            }

            // save off our sensor data, add it to queue
            currentMessage.acceleration[0] = event.acceleration.x;
            currentMessage.acceleration[1] = event.acceleration.y;
            currentMessage.acceleration[2] = event.acceleration.z;

            // zero wait time means data is dropped if queue is full,
            // i think that's okay! hopefully queue shouldn't be full
            xQueueSendToBack(GDQ.ADXL, &currentMessage, 0);

            // write data to queue

            endTime = millis();
    
            #ifdef DEBUG
                printf("ADXL Uptime: %lu [ms]\n",uptime);
                //Display the results (acceleration is measured in m/s^2)
    
                printf("X: %f [m/s^2]\n",event.acceleration.x);
                printf("Y: %f [m/s^2]\n",event.acceleration.y);
                printf("Z: %f [m/s^2]\n",event.acceleration.z);
                printf("Elapsed Time: %li\n\n", endTime - startTime);
            #endif
    
            // gives the mutex back
            xSemaphoreGive(sensor_spi_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// TODO: [NS/JL] figure out exactly raw data we have access
//               to and what it means
void BNO_task(void *pvParameter) {
    // variable declaration
    sensors_event_t orientationData, angVelocityData, magnetometerData, accelerometerData;
    uint32_t startTime, endTime;
    TickType_t uptime;
    BNOMessage_t currentMessage = {0};

    while (1) {
        // TODO: [NS/JL] assess if we actually need to gather all this data from the sensor
        // if we can't get BNO data, end task early and schedule again after 1 tick
        if (!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
            vTaskDelay(1);
        }
        if (!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
            vTaskDelay(1);
        }
        if (!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER)) {
            vTaskDelay(1);
        }
        if (!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER)) {
            vTaskDelay(1);
        }
        
        // gather function start time
        startTime = millis();
        uptime = xTaskGetTickCount();
        
        imu::Quaternion quat = BNO.getQuat();

        //save sensor data and add it to queue
        currentMessage.quaternion[0] = quat.w();
        currentMessage.quaternion[1] = quat.x();
        currentMessage.quaternion[2] = quat.y();
        currentMessage.quaternion[3] = quat.z();

        currentMessage.euler_orientation[0] = angVelocityData.gyro.x;
        currentMessage.euler_orientation[1] = angVelocityData.gyro.y;
        currentMessage.euler_orientation[2] = angVelocityData.gyro.z;

        currentMessage.magnetometer[0] = magnetometerData.magnetic.x;
        currentMessage.magnetometer[1] = magnetometerData.magnetic.y;
        currentMessage.magnetometer[2] = magnetometerData.magnetic.z;

        currentMessage.acceleration[0] = accelerometerData.acceleration.x;
        currentMessage.acceleration[1] = accelerometerData.acceleration.y;
        currentMessage.acceleration[2] = accelerometerData.acceleration.z;

        //write message to queue
        xQueueSendToBack(GDQ.BNO, &currentMessage, 0);

        endTime = millis();

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

        // vTaskDelay(pdMS_TO_TICKS(20 - (endTime - startTime)));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void LSM_task(void *pvParameter) {

    LSMMessage_t currentMessage = {0};

    TickType_t uptime; 
    uint32_t startTime;
    uint32_t endTime;

    //Sensor events 
    sensors_event_t accel; 
    sensors_event_t gyro; 
    sensors_event_t temp; 

    while (1) {
        // attempts to retrieve mutex
        if(xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE){

            uptime = xTaskGetTickCount(); 
            startTime = millis(); 
            endTime = millis();
    
            // if LSM read fails, return mutex, and schedule task again after 1 tick
            if(!LSM.getEvent(&accel, &gyro, &temp)) {
                xSemaphoreGive(sensor_spi_mutex);
                vTaskDelay(1);
            }

            // adds data to LSM struct
            currentMessage.acceleration[0] = accel.acceleration.x;
            currentMessage.acceleration[1] = accel.acceleration.y;
            currentMessage.acceleration[2] = accel.acceleration.z;

            currentMessage.gyro[0] = gyro.gyro.x;
            currentMessage.gyro[1] = gyro.gyro.y;
            currentMessage.gyro[2] = gyro.gyro.z;

            // remove comment marks to include temperate
            //currentMessage.temp = temp;

            // writes data to queue
            xQueueSendToBack(GDQ.LSM, &currentMessage, 0);


    
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
                printf("Elapsed Time: %li\n\n", endTime - startTime);
    
            #endif
            
            // give back mutex
            xSemaphoreGive(sensor_spi_mutex);
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
    BMPMessage_t currentMessage = {0};
    while(1) {
        // attempts to retrieve mutex
        if(xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE){

            // time system
            startTime = millis();
            upTime = xTaskGetTickCount();
            bmp_up = BMP.performReading();
    
            if(bmp_up) {
                bmp_temp = BMP.temperature;
                bmp_press = BMP.pressure;
                bmp_alt = BMP.readAltitude(1013.25f);
    
                // TODO: [NS] add calibration
                
                // adds data to struct
                currentMessage.temp = bmp_temp;
                currentMessage.pressure = bmp_press;
                currentMessage.alititude = bmp_alt;

                // writes data to queue
                xQueueSendToBack(GDQ.BMP, &currentMessage, 0);
    
                #ifdef DEBUG
                    printf("BMP reporting OK!\n");
                    printf("BMP Temp: %f\n", bmp_temp);
                    printf("BMP Press: %f\n", bmp_press);
                    printf("BMP Alt: %f\n", bmp_alt);
                    printf("Uptime [ms/ticks]: %lu\n\n", upTime);
                    endTime = millis();
                    printf("Elapsed Time: %li\n\n\n", endTime - startTime);
                #endif
            } else {
                #ifdef DEBUG
                    printf("BMP reporting NOT OK!\n\n");
                #endif
            }
    
            xSemaphoreGive(sensor_spi_mutex);//gives back mutex
    
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
