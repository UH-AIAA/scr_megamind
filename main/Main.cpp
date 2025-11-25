// Note: Current version is not calibrated. It would be a best practice to calibrate data during launch day
// so it returns precise values. But do calibrate when write the state machine code.

///: defnition comments
//:  important comments

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "driver/gpio.h"

/// SPI imports, I^2C, and UART Imports
#include "SPI.h"
#include "Arduino.h"
#include "Adafruit_BMP5xx.h"
#include "Adafruit_ADXL375.h"
#include "Adafruit_LSM6DSO32.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_GPS.h"
#include "LoRa.h"
#include "SD.h"

/// Sensor SPI init
#define SPI_SCLK_PIN 12
#define SPI_MISO_PIN 13 //SDO
#define SPI_MOSI_PIN 11 //SDA
#define SPI_MAX_TRSZ 4096

/// SD+LoRa SPI Init
#define VSPI_SCLK_PIN 18
#define VSPI_MISO_PIN 17
#define VSPI_MOSI_PIN 16

/// I^2C Init
#define I2C_SDA 8
#define I2C_SCL 9

/// CS definitions
#define BMP581_CS 10
#define ADXL375_CS 5
#define LSM6DSO32_CS 4
#define LORA_CS 7
#define SD_CS 20

/// Lo-Ra Control Pins
#define LORA_RST 21
#define LORA_IRQ 19
#define LORA_FREQ 915E6

/// Debug control definitions
#define DEBUG

/// Calibrated data struct
typedef struct{
    float bmp_temp, bmp_press, bmp_alt = 0;
    float adxl_acc_x, adxl_acc_y, adxl_acc_z, adxl_temp = 0;
    float lsm_acc_x, lsm_acc_y, lsm_acc_z,
          lsm_gyro_x, lsm_gyro_y, lsm_gyro_z,
          lsm_temp = 0;
    float bno_quar_w, bno_quar_x, bno_quar_y, bno_quar_z,
          bno_acc_x,  bno_acc_y,  bno_acc_z,
          bno_gyro_x, bno_gyro_y, bno_gyro_z,
          bno_mag_x,  bno_mag_y,  bno_mag_z,
          bno_ori_x,  bno_ori_y,  bno_ori_z = 0;
    float gps_sats, gps_lat, gps_long, gps_alt = 0;
} OutputData_t; 

/// Pointers to hold global objects address
typedef struct{
    OutputData_t    *pOutputData;
    sensors_event_t *pEventADXL,
                    *pEventLSM_accel,
                    *pEventLSM_gyro,
                    *pEventLSM_temp,
                    *pOrientation,
                    *pAngVelocity,
                    *pMagnetometer,
                    *pAccelerometer;
    imu::Quaternion *pQuaternion;
}TaskParams_t; 

/*
@brief 
    gOutputData:                   global Output object for all sensors
    gEventADXL :                   global output data for ADXL375
    gEventLSM_ :                   global output data for LSM 
    gOrientation to gAcceleromter: global output for BNO
    gQuarternion:                 quaternion output for BNO
*/
OutputData_t    gOutputData; 
sensors_event_t gEventADXL; 
sensors_event_t gEventLSM_accel, gEventLSM_gyro, gEventLSM_temp; 
sensors_event_t gOrientation, gAngVelocity, gMagnetometer, gAccelerometer; 
imu::Quaternion gQuaternion; 

/*
@brief 
    upTime:                       Current Output time [ms] of each sensor
    gSpiMutex :                   SPI bus mutex to manage protocol traffic
    gI2cMutex :                   I2C bus mutex to manage protocol traffic
*/
uint32_t upTime;             
SemaphoreHandle_t gSpiMutex; 
SemaphoreHandle_t gI2cMutex; 
bool gHasSD = false;
const int MAX_GPS_BYTES_PER_LOOP = 64;  // tune as needed

//Sensors Object Instantiation
Adafruit_BMP5xx     BMP;
Adafruit_ADXL375    ADXL(ADXL375_CS, &SPI);
Adafruit_LSM6DSO32  LSM;
Adafruit_BNO055     BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS        GPS(&Wire);
SPIClass            SPI2(HSPI);
File                sdData;


void init_spi() {
    /// init SPI bus for BMP + ADXL + LSM
    SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1);
    BMP.begin(BMP581_CS, &SPI);
    ADXL.begin();
    LSM.begin_SPI(LSM6DSO32_CS, &SPI);

    /// init SPI bus for SD + Lora
    SPI2.begin(VSPI_SCLK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN, -1);
    gHasSD = SD.begin(SD_CS, SPI2, 20000000); /// 20MHz SD SPI bus
    if (!gHasSD) {
        Serial.println("SD init fail");
    } else {
        Serial.println("SD init OK");
    }
}

void init_I2C() {
    ///init I2C bus for BNO + GPS
    Wire.begin(I2C_SDA, I2C_SCL);
    BNO.begin();
}

void Core0_task(void *pvParameter);
void Core1_task1(void *pvParameter);
void Core1_task2(void *pvParameter);

extern "C" void app_main()
{
    /// init Arduino Framework from ESP HAL
    initArduino();

    /// init SPI buses
    init_spi();

    /// init I^2C bus
    init_I2C();

    /// dump GPIO config
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    /// Create New Mutex
    gSpiMutex = xSemaphoreCreateMutex();
    gI2cMutex = xSemaphoreCreateMutex();

    /// Pointer holder heap allocation
    TaskParams_t *pParams    = (TaskParams_t*)malloc(sizeof(TaskParams_t));
    /// BMP+GPS object address
    pParams->pOutputData     = &gOutputData;
    /// ADXL object address
    pParams->pEventADXL      = &gEventADXL;
    /// LSM object address
    pParams->pEventLSM_accel = &gEventLSM_accel;
    pParams->pEventLSM_gyro  = &gEventLSM_gyro;
    pParams->pEventLSM_temp  = &gEventLSM_temp;
    /// BNO object address
    pParams->pOrientation    = &gOrientation;
    pParams->pAngVelocity    = &gAngVelocity;
    pParams->pMagnetometer   = &gMagnetometer;
    pParams->pAccelerometer  = &gAccelerometer;
    pParams->pQuaternion     = &gQuaternion;

    xTaskCreatePinnedToCore(Core0_task, "Core0_task", 5000, (void*)pParams,
                            1, NULL, 0);

    xTaskCreatePinnedToCore(Core1_task1, "Core1_task1", 5000, (void*)pParams,
                            2, NULL, 1);

    xTaskCreatePinnedToCore(Core1_task2, "Core1_task2", 5000, (void*)pParams,
                            1, NULL, 1);
}

void Core0_task(void *pvParameter) {
    // Using pointers here to help refactor code faster later
    /// pointers holder 
    TaskParams_t    *pTask           = (TaskParams_t*)pvParameter;
    /// BMP pointer
    OutputData_t    *pOutputData     = pTask->pOutputData;
    ///ADXL pointer
    sensors_event_t *pEventADXL      = pTask->pEventADXL;
    ///LSM pointer
    sensors_event_t *pEventLSM_accel = pTask->pEventLSM_accel;
    sensors_event_t *pEventLSM_gyro  = pTask->pEventLSM_gyro;
    sensors_event_t *pEventLSM_temp  = pTask->pEventLSM_temp;

    while(1)
    {
      /// BMP functions + calibrations
      if (xSemaphoreTake(gSpiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          upTime = xTaskGetTickCount();
          bool bmp_ok = BMP.performReading();
          xSemaphoreGive(gSpiMutex);

        if (bmp_ok) {
            // calibrate here & save to data struct 
            pOutputData->bmp_temp  = BMP.temperature-0.8;
            pOutputData->bmp_press = BMP.pressure;
            pOutputData->bmp_alt   = BMP.readAltitude(1013.25f) + 25.85;

          #ifdef DEBUG
              printf("BMP Up Time: %lu [ms]\n", (unsigned long)upTime);
              printf("BMP Temp: %f\n", pOutputData->bmp_temp);
              printf("BMP Press: %f\n", pOutputData->bmp_press);
              printf("BMP Alt: %f\n\n", pOutputData->bmp_alt);
          #endif
          }
      } else {
          #ifdef DEBUG
              printf("No token BMP!!!\n");
          #endif
      }
          

      /// ADXL function + calibrations
      if (xSemaphoreTake(gSpiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        upTime = xTaskGetTickCount();
        bool adxl_ok = ADXL.getEvent(pEventADXL);
        xSemaphoreGive(gSpiMutex);

        if (adxl_ok) {
            // calibrate here & save to data struct 
            pOutputData->adxl_acc_x = pEventADXL->acceleration.x - 9.11;
            pOutputData->adxl_acc_y = pEventADXL->acceleration.x - 1.44;
            pOutputData->adxl_acc_z = pEventADXL->acceleration.x - 3.344;

            #ifdef DEBUG
                    printf("ADXL Up Time: %lu [ms]\n", (unsigned long)upTime);
                    printf("ADXL accel X: %f [m/s^2]\n", pOutputData->adxl_acc_x);
                    printf("ADXL accel Y: %f [m/s^2]\n", pOutputData->adxl_acc_y);
                    printf("ADXL accel Z: %f [m/s^2]\n\n", pOutputData->adxl_acc_z);
            #endif
        }
      } else {
          #ifdef DEBUG
              printf("No token ADXL!!!\n");
          #endif
      }


      /// LSM function + calibrations
      if (xSemaphoreTake(gSpiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        upTime = xTaskGetTickCount();
        bool lsm_ok = LSM.getEvent(pEventLSM_accel, pEventLSM_gyro, pEventLSM_temp);
        xSemaphoreGive(gSpiMutex);

        if (lsm_ok) {
            // calibrate here & save to data struct 
            pOutputData->lsm_acc_x  = pEventLSM_accel->acceleration.x + 0.003;
            pOutputData->lsm_acc_y  = pEventLSM_accel->acceleration.y + 0.003;
            pOutputData->lsm_acc_z  = pEventLSM_accel->acceleration.z - 9.631;
            pOutputData->lsm_gyro_x = pEventLSM_gyro->gyro.x + 0.003;
            pOutputData->lsm_gyro_y = pEventLSM_gyro->gyro.y+0.115;
            pOutputData->lsm_gyro_z = pEventLSM_gyro->gyro.z;
            pOutputData->lsm_temp   = pEventLSM_temp->temperature-2.5;

            #ifdef DEBUG
            printf("LSM Up Time: %lu [ms]\n", (unsigned long)upTime);
            printf("LSM Accel X: %f [m/s^2]\n", pEventLSM_accel->acceleration.x);
            printf("LSM Accel Y: %f [m/s^2]\n", pEventLSM_accel->acceleration.y);
            printf("LSM Accel Z: %f [m/s^2]\n", pEventLSM_accel->acceleration.z);

            printf("LSM Gyro X: %f\n", pEventLSM_gyro->gyro.x);
            printf("LSM Gyro Y: %f\n", pEventLSM_gyro->gyro.y);
            printf("LSM Gyro Z: %f\n", pEventLSM_gyro->gyro.z);

            printf("LSM Temp: %f\n\n", pEventLSM_temp->temperature);
            #endif
        }
      } else {
          #ifdef DEBUG
              printf("No token LSM!!!\n");
          #endif
      }

      vTaskDelay(pdMS_TO_TICKS(10)); // Stable, Unoptimized

    }
}

void Core1_task1(void *pvParameter) {
    // Using pointers here to help refactor code faster later
    /// Pointers holder
    TaskParams_t *pTask       = (TaskParams_t *)pvParameter;
    /// GPS pointer
    OutputData_t *pOutputData = pTask->pOutputData;
    /// BNO pointer
    sensors_event_t *pEventBNO_ori   = pTask->pOrientation;
    sensors_event_t *pEventBNO_angVel= pTask->pAngVelocity;
    sensors_event_t *pEventBNO_mag   = pTask->pMagnetometer;
    sensors_event_t *pEventBNO_accel = pTask->pAccelerometer;
    imu::Quaternion *pQuaternion     = pTask->pQuaternion;

    while (1) {

        ///BNO function 
        bool orient_ok = false;
        bool gyro_ok   = false;
        bool mag_ok    = false;
        bool accel_ok  = false;

        #ifdef DEBUG
            printf("[BNO] trying to take I2C mutex...\n");
        #endif

        if (xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            upTime = xTaskGetTickCount();
            orient_ok = BNO.getEvent(pEventBNO_ori,    Adafruit_BNO055::VECTOR_EULER);
            gyro_ok   = BNO.getEvent(pEventBNO_angVel, Adafruit_BNO055::VECTOR_GYROSCOPE);
            mag_ok    = BNO.getEvent(pEventBNO_mag,    Adafruit_BNO055::VECTOR_MAGNETOMETER);
            accel_ok  = BNO.getEvent(pEventBNO_accel,  Adafruit_BNO055::VECTOR_ACCELEROMETER);
            // quaternion also uses I2C, so keep it inside the lock
            *pQuaternion = BNO.getQuat();
            xSemaphoreGive(gI2cMutex);

            bool bno_ok = orient_ok && gyro_ok && mag_ok && accel_ok;
            if (bno_ok) {
                /// calibrate here & save to data struct 
                pOutputData->bno_quar_w = pQuaternion->w();
                pOutputData->bno_quar_x = pQuaternion->x();
                pOutputData->bno_quar_y = pQuaternion->y();
                pOutputData->bno_quar_z = pQuaternion->z();

                pOutputData->bno_ori_x  = pEventBNO_ori->orientation.x;
                pOutputData->bno_ori_y  = pEventBNO_ori->orientation.y;
                pOutputData->bno_ori_z  = pEventBNO_ori->orientation.z;

                pOutputData->bno_gyro_x = pEventBNO_angVel->gyro.x;
                pOutputData->bno_gyro_y = pEventBNO_angVel->gyro.y;
                pOutputData->bno_gyro_z = pEventBNO_angVel->gyro.z;

                pOutputData->bno_mag_x  = pEventBNO_mag->magnetic.x;
                pOutputData->bno_mag_y  = pEventBNO_mag->magnetic.y;
                pOutputData->bno_mag_z  = pEventBNO_mag->magnetic.z;

                pOutputData->bno_acc_x  = pEventBNO_accel->acceleration.x;
                pOutputData->bno_acc_y  = pEventBNO_accel->acceleration.y;
                pOutputData->bno_acc_z  = pEventBNO_accel->acceleration.z - 9.35;

                #ifdef DEBUG
                        printf("BNO Uptime: %lu [ms]\n", (unsigned long)upTime);

                        printf("BNO Quaternion:\n");
                        printf("BNO quater W: %f\n",   pOutputData->bno_quar_w);
                        printf("BNO quater X: %f\n",   pOutputData->bno_quar_x);
                        printf("BNO quater Y: %f\n",   pOutputData->bno_quar_y);
                        printf("BNO quater Z: %f\n\n", pOutputData->bno_quar_z);

                        printf("BNO Orientation:\n");
                        printf("BNO Ori X: %f\n",   pOutputData->bno_ori_x);
                        printf("BNO Ori Y: %f\n",   pOutputData->bno_ori_y);
                        printf("BNO Ori Z: %f\n\n", pOutputData->bno_ori_z);

                        printf("BNO Gyro:\n");
                        printf("BNO Gyro X: %f\n",   pOutputData->bno_gyro_x);
                        printf("BNO Gyro Y: %f\n",   pOutputData->bno_gyro_y);
                        printf("BNO Gyro Z: %f\n\n", pOutputData->bno_gyro_z);

                        printf("BNO Magnometer:\n");
                        printf("BNO Mag X: %f\n", pOutputData->bno_mag_x);
                        printf("BNO Mag Y: %f\n", pOutputData->bno_mag_y);
                        printf("BNO Mag Z: %f\n\n", pOutputData->bno_mag_z);

                        printf("BNO Accelerometer:\n");
                        printf("BNO Accel X: %f\n", pOutputData->bno_acc_x);
                        printf("BNO Accel Y: %f\n", pOutputData->bno_acc_y);
                        printf("BNO Accel Z: %f\n\n", pOutputData->bno_acc_z);
                #endif
                
            }
        } else {

            #ifdef DEBUG
                printf("No token BNO!!!\n");
            #endif

        }



        /// GPS function
        int bytes_processed = 0;
        bool     got_fix     = false;
        uint32_t cycle_start = millis();
        uint32_t timeout     = cycle_start + 200;  // ~200 ms window for this cycle

        while ((millis() < timeout) && !got_fix) { // Might be redudant since data is capped (Stable - Unoptimized)
            // Try to grab I2C bus briefly

            #ifdef DEBUG
                printf("GPS starts running***************\n");
            #endif

            if (xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

                #ifdef DEBUG
                    printf("GPS got Mutex!!!!!!!!!!!!!!!!!!!!!!!!!!***************\n");
                #endif

                upTime = xTaskGetTickCount();

                // Drain only certain bytes currently in the GPS buffer to PREVENT INFINITE LOOP (Stable-Unoptimized)
                while (GPS.available() && bytes_processed < MAX_GPS_BYTES_PER_LOOP) {

                    #ifdef DEBUG
                        printf("GPS available---------------------------------------\n");
                    #endif    

                    GPS.read();  // reads ONE byte

                    if (GPS.newNMEAreceived()) {
                        // Parse only good sentences
                        if (!GPS.parse(GPS.lastNMEA())) {

                            #ifdef DEBUG
                                printf("GPS bad sentence @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
                            #endif

                            continue;  // bad sentence, skip
                        }

                        // Check for a valid fix with satellites
                        if (GPS.fix && GPS.satellites > 0) {

                            #ifdef DEBUG
                                printf("GPS saving data to struct\n");
                            #endif

                            // Save into shared struct
                            pOutputData->gps_sats = GPS.satellites;
                            pOutputData->gps_lat  = GPS.latitude;
                            pOutputData->gps_long = GPS.longitude;
                            pOutputData->gps_alt  = GPS.altitude;

                            #ifdef DEBUG
                                printf("GPS: fix OK\n");
                                printf("GPS Uptime: %lu [ticks]\n", (unsigned long)upTime);
                                printf("Satellites: %d\n", GPS.satellites);
                                printf("Latitude:  %f %c\n", GPS.latitude,  GPS.lat);
                                printf("Longitude: %f %c\n", GPS.longitude, GPS.lon);
                                printf("Altitude:  %f [m]\n\n", GPS.altitude);
                            #endif

                            got_fix = true;
                            break;  // break while(GPS.available())
                        }
                    }
                    bytes_processed++;
                }

                #ifdef DEBUG
                    printf("GPS releasing token----------------------------------\n");
                #endif

                xSemaphoreGive(gI2cMutex);
            } else {

                #ifdef DEBUG
                    printf("No token GPS!!!\n");
                #endif

            }
        }

        #ifdef DEBUG
            if (!got_fix) {
                printf("GPS: no valid fix this cycle\n");
            }
        #endif
        

        vTaskDelay(pdMS_TO_TICKS(110)); // Stable, Unoptimized
    }
}


void Core1_task2(void *pvParameter) {
    // Using pointers here to help refactor code faster later
    /// Pointers holder
    TaskParams_t *pTask       = (TaskParams_t *)pvParameter;

    while (1) {
        /// SD card saving (Append data) - get all data
        // only txt works. csv triggers watchdog
        if (gHasSD){
            sdData = SD.open("/SD_data.txt", FILE_APPEND);
            if (sdData) {
                if (sdData.size() >= 1000 && sdData.size() < 1200 ) { // skip <10 log lines and print header
                    sdData.println(
                                "time_ms,"
                                "bmp_temp,bmp_press,bmp_alt,"
                                "adxl_acc_x,adxl_acc_y,adxl_acc_z,"
                                "lsm_acc_x,lsm_acc_y,lsm_acc_z,"
                                "lsm_gyro_x,lsm_gyro_y,lsm_gyro_z,lsm_temp,"
                                "bno_quar_w, bno_quar_x, bno_quar_y, bno_quar_z,"
                                "bno_acc_x,bno_acc_y,bno_acc_z,"
                                "bno_gyro_x,bno_gyro_y,bno_gyro_z,"
                                "bno_mag_x,bno_mag_y,bno_mag_z,"
                                "bno_ori_x,bno_ori_y,bno_ori_z,"
                                "gps_sats,gps_lat,gps_long,gps_alt"
                                );
                }

                /// CurrentTime 
                sdData.print(millis()); sdData.print(',');

                /// BMP
                sdData.print(gOutputData.bmp_temp);   sdData.print(',');
                sdData.print(gOutputData.bmp_press);  sdData.print(',');
                sdData.print(gOutputData.bmp_alt);    sdData.print(',');

                /// ADXL
                sdData.print(gOutputData.adxl_acc_x); sdData.print(',');
                sdData.print(gOutputData.adxl_acc_y); sdData.print(',');
                sdData.print(gOutputData.adxl_acc_z); sdData.print(',');

                /// LSM (accel + gyro + temp)
                sdData.print(gOutputData.lsm_acc_x);  sdData.print(',');
                sdData.print(gOutputData.lsm_acc_y);  sdData.print(',');
                sdData.print(gOutputData.lsm_acc_z);  sdData.print(',');

                sdData.print(gOutputData.lsm_gyro_x); sdData.print(',');
                sdData.print(gOutputData.lsm_gyro_y); sdData.print(',');
                sdData.print(gOutputData.lsm_gyro_z); sdData.print(',');
                sdData.print(gOutputData.lsm_temp);   sdData.print(',');

                /// BNO (quar, acc, gyro, mag, ori, temp)
                sdData.print(gOutputData.bno_quar_w);  sdData.print(',');
                sdData.print(gOutputData.bno_quar_x);  sdData.print(',');
                sdData.print(gOutputData.bno_quar_y);  sdData.print(',');
                sdData.print(gOutputData.bno_quar_z);  sdData.print(',');

                sdData.print(gOutputData.bno_acc_x);  sdData.print(',');
                sdData.print(gOutputData.bno_acc_y);  sdData.print(',');
                sdData.print(gOutputData.bno_acc_z);  sdData.print(',');

                sdData.print(gOutputData.bno_gyro_x); sdData.print(',');
                sdData.print(gOutputData.bno_gyro_y); sdData.print(',');
                sdData.print(gOutputData.bno_gyro_z); sdData.print(',');

                sdData.print(gOutputData.bno_mag_x);  sdData.print(',');
                sdData.print(gOutputData.bno_mag_y);  sdData.print(',');
                sdData.print(gOutputData.bno_mag_z);  sdData.print(',');

                sdData.print(gOutputData.bno_ori_x);  sdData.print(',');
                sdData.print(gOutputData.bno_ori_y);  sdData.print(',');
                sdData.print(gOutputData.bno_ori_z);  sdData.print(',');

                /// GPS
                sdData.print(gOutputData.gps_sats);   sdData.print(',');
                sdData.print(gOutputData.gps_lat);    sdData.print(',');
                sdData.print(gOutputData.gps_long);   sdData.print(',');
                sdData.print(gOutputData.gps_alt);

                sdData.println(); 

                #ifdef DEBUG
                    printf("Write Success!!\n");
                #endif

                sdData.flush();
            } else {

                #ifdef DEBUG
                    printf("Cant open file!!!\n");
                #endif

            }
        } else {

            #ifdef DEBUG
                printf("No SD card found!!!\n");
            #endif

        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Stable, Unoptimized
    }
}