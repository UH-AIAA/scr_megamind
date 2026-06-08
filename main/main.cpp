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
#include "RTClib.h"

// SRAD Imports
#include "megamind.h"
#include "megamind_pins.h"
#include "megamind_const.h"

// Debug control definitions
// #define DEBUG

// Chip Object Instantiation
Adafruit_BMP5xx BMP;
Adafruit_ADXL375 ADXL(ADXL375_CS, &SPI);
Adafruit_LSM6DSO32 LSM;
Adafruit_BNO055 BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS GPS(&Wire);
SPIClass SPI2(HSPI);
TwoWire RTCWire = TwoWire(1);
RTC_DS1307 RTC;
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
uint8_t fsmState = 0;
float apogeeEstimate = 0;

inline spinlock()
{
    // ensure side effect so compiler does not optimize away the infinite loop
    volatile int i{};
    while (true)
    {
        i++;
    }
}

void init_spi()
{
    // SPI2.begin(VSPI_SCLK_PIN,
    //            VSPI_MISO_PIN,
    //            VSPI_MOSI_PIN,
    //            -1);

    // printf("Trying SD.begin...\n");

    // bool ok = SD.begin(SD_CS, SPI2, 20000000);

    // printf("SD.begin returned %d\n", ok);

    // while (true);

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
    BMP.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_1);

    LSM.setAccelDataRate(LSM6DS_RATE_104_HZ); //gives accelerometer data every 9.6ms

    // use LSM for fine-tuned coast data, set to smallest interval(+- 4Gs)
    LSM.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    // additionally set roll rate in degs/sec, pulled from rocksim pro data
    LSM.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS); // guess, waiting on data from dedah


    // max G-force ~= 14 Gs, setting to +- 16Gs.
    ADXL.setRange(ADXL343_RANGE_16_G);
    
    // SD setup
    if (!SPI2.begin(VSPI_SCLK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN, -1))
    {
        printf("SPI2 failed\n");
        spinlock();
    }

    if (!SD.begin(SD_CS, SPI2, 20E6))   // TODO: [NS] make this a #define
    {
        printf("SD never began!\n");
        spinlock();
    }

    char rtctime[64];
    DateTime now = RTC.now();
    snprintf(
        rtctime,
        sizeof(rtctime),
        "%04d-%02d-%02d-%02d-%02d-%02d",
        now.year(),
        now.month(),
        now.day(),
        now.hour() - 60,
        now.minute(),
        now.second()
    );

    // Create datalogging file with a unique name
    char csvfilename[17] = "/FL0.txt";
    for(uint32_t i = 0; SD.exists(csvfilename); i++)
    {
        sprintf(csvfilename, "/FL%ld.txt", i);  // Increment filename if it already exists
    }

    sdData = SD.open(csvfilename, FILE_WRITE);
    if (!sdData)
    {
        printf("SD FILE FAILED\n");
        spinlock();
    }
    sdData.println(rtctime);
    sdData.println(header);

    // LORA setup (Thanh's work!)
    LoRa.setSPI(SPI2);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    if (!LoRa.begin(LORA_FREQ))
    {
        printf("LoRa.begin failed!\n");
        spinlock();
    }
}

void init_I2C()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!RTCWire.begin(RTC_SDA, RTC_SCL))
    {
        printf("RTC wire failed to init\n");
        spinlock();
    }

    // GPS Setup
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);

    // BNO begin
    // TODO: [NS] - figure out how to set BNO to manual
    BNO.begin();
    BNO.setExtCrystalUse(true);

    // RTC!
    if (!RTC.begin(&RTCWire))
    {
        printf("RTC failed to start!\n");
        spinlock();
    }

    if (!RTC.isrunning())
    {
        printf("RTC not running, time not set!\n");
        spinlock();
    }
}

// init queues
void init_GDQ()
{
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

    // init I^2C bus
    init_I2C();

    // init SPI buses
    init_spi();

    // init GDQ
    init_GDQ();

    // dump GPIO config
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    // initialize Mutex
    sensor_spi_mutex = xSemaphoreCreateMutex();
    sd_lora_spi_mutex = xSemaphoreCreateMutex();  

    // TODO: [add calibration functions here for LSM] [NS]
    while (!calibrateIMUs(&ADXL, &LSM, ADXL_ACCEL_BIAS, LSM_ACCEL_BIAS, LSM_GYRO_BIAS, NUM_CALIBRATION_SAMPLES, CALIBRATION_DIVERGE_THRESH))
    {
        printf("IMU Calibration Failed!\n");
    }

    printf("LSM X, Y, Z bias: %f, %f, %f\n", LSM_ACCEL_BIAS[0], LSM_ACCEL_BIAS[1], LSM_ACCEL_BIAS[2]);
    printf("ADXL X, Y, Z bias: %f, %f, %f\n", ADXL_ACCEL_BIAS[0], ADXL_ACCEL_BIAS[1], ADXL_ACCEL_BIAS[2]);


    // TODO: [JF] call altimeter calibration function here!
    while (!calibrateAltimeter(&BMP, &BMP_ALT_BIAS, NUM_CALIBRATION_SAMPLES, CALIBRATION_DIVERGE_THRESH))
    {
        printf("Altitude Calibration Failed!\n");
    }
    printf("alt bias: %f\n\n", BMP_ALT_BIAS);

    // https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html
    // BaseType_t xTaskCreatePinnedToCore(
    //       TaskFunction_t pvTaskCode
    //     , const char *const pcName
    //     , const uint32_t usStackDepth
    //     , void *const pvParameters
    //     , const UBaseType_t uxPriority
    //     , TaskHandle_t *const pvCreatedTask
    //     , const BaseType_t xCoreID
    // );

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

    /* LORA Task */
    {
        TaskFunction_t pvTaskCode{&LORA_task};
        const char* const pcName{"LORA_task"};
        const uint32_t usStackDepth{10'000};
        void* const pvParameters{nullptr};
        const UBaseType_t uxPriority{5};
        TTaskHandle_t* const pvCreatedTask{&LORA_handle};
        const BaseType_t xCoreID{1};
        
        xTaskCreatePinnedToCore(
            pvTaskCode,
            pcName,
            usStackDepth,
            pvParameters,
            uxPriority,
            pvCreatedTask,
            xCoreID
        );
    }

    /* GPS Task */
    {
        TaskFunction_t pvTaskCode{GPS_task};
        const char* const pcName{"GPS Task"};
        const uint32_t usStackDepth{8000};
        void* const pvParameters{nullptr};
        const UBaseType_t uxPriority{3};
        TTaskHandle_t* const pvCreatedTask{nullptr};
        const BaseType_t xCoreID{1};

        xTaskCreatePinnedToCore(
            pvTaskCode,
            pcName,
            usStackDepth,
            pvParameters,
            uxPriority,
            pvCreatedTask,
            xCoreID
        );
    }

    // NOTE: temporarily commented out while waiting for new SD chip
    /* SD Task */
    {
        TaskFunction_t pvTaskCode{SD_task};
        const char* const pcName{"SD_task"};
        const uint32_t usStackDepth{8000};
        void* const pvParameters{nullptr};
        const UBaseType_t uxPriority{4};
        TTaskHandle_t* const pvCreatedTask{nullptr};
        const BaseType_t xCoreID{1};

        xTaskCreatePinnedToCore(
            pvTaskCode,
            pcName,
            usStackDepth,
            pvParameters,
            uxPriority,
            pvCreatedTask,
            xCoreID
        );
    }

    /* ADXL Task */
    {
        TaskFunction_t pvTaskCode{&ADXL_task};
        const char* const pcName{"ADXL_task"};
        const uint32_t usStackDepth{5000};
        void* const pvParameters{nullptr};
        const UBaseType_t uxPriority{5};
        TTaskHandle_t* const pvCreatedTask{nullptr};
        const BaseType_t xCoreID{0};

        xTaskCreatePinnedToCore(
            pvTaskCode,
            pcName,
            usStackDepth,
            pvParameters,
            uxPriority,
            pvCreatedTask,
            xCoreID
        );
    }

    /* BNO Task */
    {
        TaskFunction_t pvTaskCode{&BNO_task};
        const char* const pcName{"BNO_task"};
        const uint32_t usStackDepth{5000};
        void* const pvParameters{nullptr};
        const UBaseType_t uxPriority{2};
        TTaskHandle_t* const pvCreatedTask{nullptr};
        const BaseType_t xCoreID{0};

        xTaskCreatePinnedToCore(
            pvTaskCode,
            pcName,
            usStackDepth,
            pvParameters,
            uxPriority,
            pvCreatedTask,
            xCoreID
        );
    }

    /* LSM Task */
    {
        TaskFunction_t pvTaskCode{&LSM_task};
        const char* const pcName{"LSM_task"};
        const uint32_t usStackDepth{5000};
        void* const pvParameters{nullptr};
        const UBaseType_t uxPriority{3};
        TTaskHandle_t* const pvCreatedTask{nullptr};
        const BaseType_t xCoreID{0};

        xTaskCreatePinnedToCore(
            pvTaskCode,
            pcName,
            usStackDepth,
            pvParameters,
            uxPriority,
            pvCreatedTask,
            xCoreID
        );
    }

    /* BMP Task */
    {
        TaskFunction_t pvTaskCode{&BMP_task};
        const char* const pcName{"BMP_task"};
        const uint32_t usStackDepth{ 5000};
        void* const pvParameters{nullptr};
        const UBaseType_t uxPriority{4};
        TTaskHandle_t* const pvCreatedTask{nullptr};
        const BaseType_t xCoreID{0};

        xTaskCreatePinnedToCore(
            pvTaskCode,
            pcName,
            usStackDepth,
            pvParameters,
            uxPriority,
            pvCreatedTask,
            xCoreID
        );
    }
}

void GPS_task(void *pvParameter)
{
    GDQMessage_t currentMessage = {
        .sensor = SENSOR_GPS,
    };

    const TickType_t period = pdMS_TO_TICKS(200);
    TickType_t lastWake = xTaskGetTickCount();

    while (true)
    {
        uint32_t startms = esp_timer_get_time();
        xTaskNotifyGive(LORA_handle);
        uint32_t timeout = startms + 200000;

        while (esp_timer_get_time() < timeout)
        {
            while (GPS.available() && esp_timer_get_time() < timeout)
            {
                GPS.read();
                //choke point is from GPS.read();
                //can only read 1 byte at a time
                if (GPS.newNMEAreceived())
                {
                    if (!GPS.parse(GPS.lastNMEA()))
                    {   
                        continue;
                    }

                    if (GPS.fix && GPS.satellites > 0)
                    {
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

void ADXL_task(void *pvParameter)
{
    // variable declarations
    GDQMessage_t currentMessage = {
        .sensor = SENSOR_ADXL,
    };

    uint32_t startTime{}, endTime{};
    TickType_t uptime{};

    static bool adxl_up;

    while (true)
    {
        // gather what time the function started
        startTime = esp_timer_get_time();
        uptime = xTaskGetTickCount();

        // attempt to take mutex, code inside blocked until mutex is released
        if(xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE)
        {
            // if success,
            adxl_up = ReadADXL(&ADXL, &currentMessage, ADXL_ACCEL_BIAS);
            // gives the mutex back
            xSemaphoreGive(sensor_spi_mutex);
        }

        if (adxl_up)
        {
            currentMessage.ADXLMessage.time = startTime;
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
void BNO_task(void *pvParameter)
{
    // variable declaration
    uint32_t startTime, endTime;
    TickType_t uptime;
    GDQMessage_t currentMessage = {
        .sensor = SENSOR_BNO,
    };

    while (true)
    {
        // TODO: [NS/JL] assess if we actually need to gather all this data from the sensor
        // if we can't get BNO data, end task early and schedule again after 1 tick

        // gather function start time
        startTime = esp_timer_get_time();
        uptime = xTaskGetTickCount();

        if (ReadBNO(&BNO, &currentMessage))
        {
            currentMessage.BNOMessage.uptime = startTime;

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


void LSM_task(void *pvParameter)
{

    GDQMessage_t currentMessage = {
        .sensor = SENSOR_LSM,
    };

    TickType_t uptime;
    uint32_t startTime;
    uint32_t endTime;

    while (true)
    {
        uptime = xTaskGetTickCount(); 
        startTime = esp_timer_get_time(); 

        // attempts to retrieve mutex
        if (xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE)
        {
            // remove comment marks to include temperate
            //currentMessage.temp = temp;
            if (ReadLSM(&LSM, &currentMessage, LSM_ACCEL_BIAS, LSM_GYRO_BIAS))
            {
                currentMessage.LSMMessage.time = startTime;
                // writes data to queue
                if (xQueueSendToBack(GDQ.SensorQueue, &currentMessage, 0) != pdTRUE)
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


void BMP_task(void *pvParameter)
{
    static bool bmp_up;
    static float bmp_temp, bmp_press, bmp_alt;
    static TickType_t upTime;
    static uint32_t startTime, endTime;
    
    //initialize struct
    GDQMessage_t currentMessage = {
        .sensor = SENSOR_BMP,
    };

    while (true)
    {
        // attempts to retrieve mutex
        if (xSemaphoreTake(sensor_spi_mutex, portMAX_DELAY) == pdTRUE)
        {
            // time system
            startTime = esp_timer_get_time();
            upTime = xTaskGetTickCount();
            bmp_up = ReadBMP(&BMP, &currentMessage, &BMP_ALT_BIAS);
            xSemaphoreGive(sensor_spi_mutex);//gives back mutex
            endTime = esp_timer_get_time();

            if (bmp_up)
            {
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
                    printf("BMP Filtered Alt: %f\n", currentMessage.BMPMessage.filteredAltitude);
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
    static int n = 0;
    static uint32_t flushCounter = 0;
    
    GDQMessage_t currentMessage;

    // TODO: [NS] make these #defines

    while (true)
    {
        // TODO: [NS] SD SPI Mutex
        // printf("started sd...\n");

        // block task until queue has data
        xQueueReceive(GDQ.SensorQueue, &currentMessage, portMAX_DELAY);
        
        // for now, run state machine in here:
        if (FSM(currentMessage, fsmState, apogeeEstimate))
        {
            // state has been updated, handle accordingly
            // if buffer full, write out
            if (!updateSDBuffer(currentMessage, &fsmState, msgBuf, &index, sizeof(msgBuf)))
            {
                xSemaphoreTake(sd_lora_spi_mutex, portMAX_DELAY);

                sdData.print(msgBuf);
                flushCounter += index;
                if (flushCounter >= 32E3)
                {
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

                // reset buffer and try again
                index = 0;
                memset(msgBuf, 0, sizeof(msgBuf));
                updateSDBuffer(currentMessage, &fsmState, msgBuf, &index, sizeof(msgBuf));
            } 
           
        }
        uint32_t startTime = esp_timer_get_time();
        TickType_t uptime = xTaskGetTickCount();

        // if buffer full, write out
        if (!updateSDBuffer(currentMessage, NULL, msgBuf, &index, sizeof(msgBuf)))
        {
            xSemaphoreTake(sd_lora_spi_mutex, portMAX_DELAY);

            sdData.print(msgBuf);
            flushCounter += index;
            if (flushCounter >= 32E3)
            {
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
            memset(msgBuf, 0, sizeof(msgBuf));
            updateSDBuffer(currentMessage, NULL, msgBuf, &index, sizeof(msgBuf));
            vTaskDelay(pdMS_TO_TICKS(20));
        } 
    }
}

void LORA_task(void *pvParameter)
{
    static TickType_t uptime;
    static uint32_t startTime, endTime;
    LORAMessage_t currentMessage;
    while (true)
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

        currentMessage.BNO_euler[0] = (int16_t)(GDQ.LatestBNOMsg.euler_orientation[0] * 1000);
        currentMessage.BNO_euler[1] = (int16_t)(GDQ.LatestBNOMsg.euler_orientation[1] * 1000);
        currentMessage.BNO_euler[2] = (int16_t)(GDQ.LatestBNOMsg.euler_orientation[2] * 1000);

        currentMessage.BNO_magnet[0] = (int16_t)(GDQ.LatestBNOMsg.magnetometer[0] * 1000);
        currentMessage.BNO_magnet[1] = (int16_t)(GDQ.LatestBNOMsg.magnetometer[1] * 1000);
        currentMessage.BNO_magnet[2] = (int16_t)(GDQ.LatestBNOMsg.magnetometer[2] * 1000);

        currentMessage.BNO_accel[0] = (int16_t)(GDQ.LatestBNOMsg.acceleration[0] * 1000);
        currentMessage.BNO_accel[1] = (int16_t)(GDQ.LatestBNOMsg.acceleration[1] * 1000);
        currentMessage.BNO_accel[2] = (int16_t)(GDQ.LatestBNOMsg.acceleration[2] * 1000);
        
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
        currentMessage.apogeeEstimate = apogeeEstimate;
        currentMessage.flightState = fsmState;

        if (xSemaphoreTake(sd_lora_spi_mutex, portMAX_DELAY) == pdTRUE)
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