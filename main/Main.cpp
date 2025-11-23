/// Note: Current version is not calibrated. It would be a best practice to calibrate data during launch day
/// so it returns precise values. But do calibrate when write the state machine code.

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
#include "SD.h"

// Sensor SPI init
#define SPI_SCLK_PIN 12
#define SPI_MISO_PIN 13 //SDO
#define SPI_MOSI_PIN 11 //SDA
#define SPI_MAX_TRSZ 4096

// SD+LoRa SPI Init
#define VSPI_SCLK_PIN 18
#define VSPI_MISO_PIN 17
#define VSPI_MOSI_PIN 16

// I^2C Init
#define I2C_SDA 8
#define I2C_SCL 9

// CS definitions
#define BMP581_CS 10
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

typedef struct{ 
    float lsm_gyro_x, lsm_gyro_y, lsm_gyro_z;
    float lsm_acc_x, lsm_acc_y, lsm_acc_z;
    float adxl_acc_x, adxl_acc_y, adxl_acc_z;
    float bno_gyro_x, bno_gyro_y, bno_gyro_z;
    float bno_acc_x, bno_acc_y, bno_acc_z;
    float bno_mag_x, bno_mag_y, bno_mag_z;
    float bno_ori_w, bno_ori_x, bno_ori_y, bno_ori_z;
    float lsm_temp, adxl_temp, bno_temp;
    float bmp_temp, bmp_press, bmp_alt;
    float gps_sats, gps_lat, gps_long, gps_alt;
}OutputData_t; //eventually to use to save to sd card and lora

typedef struct{
    OutputData_t *pOutputData;
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

OutputData_t    gOutputData; /// global Output object for sensors data
sensors_event_t gEventADXL; /// global output data for ADXL375
sensors_event_t gEventLSM_accel, gEventLSM_gyro, gEventLSM_temp; /// global output data for LSM 
sensors_event_t gOrientation, gAngVelocity, gMagnetometer, gAccelerometer; // global output for BNO
imu::Quaternion gQuaternion; //quaternion output for BNO

uint32_t upTime; /// The current time of task is running in the beginning
SemaphoreHandle_t gSpiMutex; /// SPI bus mutex to manage protocol traffic
SemaphoreHandle_t gI2cMutex; /// I2C bus mutex to manage protocol traffic

//Sensors Object Instantiation
Adafruit_BMP5xx     BMP;
Adafruit_ADXL375    ADXL(ADXL375_CS, &SPI);
Adafruit_LSM6DSO32  LSM;
Adafruit_BNO055     BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS        GPS(&Wire);
SPIClass            SPI2(HSPI);
File                sdData;


void init_spi() {
    SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1);
    BMP.begin(BMP581_CS, &SPI);
    ADXL.begin();
    LSM.begin_SPI(LSM6DSO32_CS, &SPI);
    SPI2.begin(VSPI_SCLK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN, -1);
    SD.begin(SD_CS, SPI2, 20000000); //20MHz SD SPI bus
}

void init_I2C() {
    Wire.begin(I2C_SDA, I2C_SCL);
    // BNO begin
    BNO.begin();
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

    // Create New Mutex
    gSpiMutex = xSemaphoreCreateMutex();
    gI2cMutex = xSemaphoreCreateMutex();

    TaskParams_t *pParams    = (TaskParams_t*)malloc(sizeof(TaskParams_t));
    //BMP+GPS object address
    pParams->pOutputData     = &gOutputData;
    ///ADXL object address
    pParams->pEventADXL      = &gEventADXL;
    ///LSM object address
    pParams->pEventLSM_accel = &gEventLSM_accel;
    pParams->pEventLSM_gyro  = &gEventLSM_gyro;
    pParams->pEventLSM_temp  = &gEventLSM_temp;
    ///BNO object address
    pParams->pOrientation    = &gOrientation;
    pParams->pAngVelocity    = &gAngVelocity;
    pParams->pMagnetometer   = &gMagnetometer;
    pParams->pAccelerometer  = &gAccelerometer;
    pParams->pQuaternion     = &gQuaternion;

    xTaskCreatePinnedToCore(
        Core0_task,
        "Core0_task",
        5000,
        (void*)pParams,
        1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        Core1_task,
        "Core1_task",
        5000,
        (void*)pParams,
        1,
        NULL,
        1
    );
}

void Core0_task(void *pvParameter) {
    TaskParams_t    *pTask           = (TaskParams_t*)pvParameter;
    ///BMP pointer
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
              pOutputData->bmp_temp  = BMP.temperature;
              pOutputData->bmp_press = BMP.pressure;
              pOutputData->bmp_alt   = BMP.readAltitude(1013.25f);

          #ifdef DEBUG
              printf("BMP Up Time: %lu [ms]\n", (unsigned long)upTime);
              printf("BMP Temp: %f\n", pOutputData->bmp_temp-0.8);
              printf("BMP Press: %f\n", pOutputData->bmp_press);
              printf("BMP Alt: %f\n\n", pOutputData->bmp_alt + 25.85);
          #endif
          }
      } else {
          #ifdef DEBUG
              printf("No token BMP!!!");
          #endif
      }
          

      /// ADXL function + calibrations
      if (xSemaphoreTake(gSpiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        upTime = xTaskGetTickCount();
        bool adxl_ok = ADXL.getEvent(pEventADXL);
        xSemaphoreGive(gSpiMutex);

        if (adxl_ok) {
          #ifdef DEBUG
                  printf("ADXL Up Time: %lu [ms]\n", (unsigned long)upTime);
                  printf("ADXL accel X: %f [m/s^2]\n", pEventADXL->acceleration.x-9.11);
                  printf("ADXL accel Y: %f [m/s^2]\n", pEventADXL->acceleration.y-1.44);
                  printf("ADXL accel Z: %f [m/s^2]\n\n", pEventADXL->acceleration.z-3.344);
          #endif
        }
      } else {
          #ifdef DEBUG
              printf("No token ADXL!!!");
          #endif
      }


      // LSM function + calibrations
      if (xSemaphoreTake(gSpiMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        upTime = xTaskGetTickCount();
        bool lsm_ok = LSM.getEvent(pEventLSM_accel, pEventLSM_gyro, pEventLSM_temp);
        xSemaphoreGive(gSpiMutex);

        if (lsm_ok) {
          #ifdef DEBUG
            printf("LSM Up Time: %lu [ms]\n", (unsigned long)upTime);
            printf("LSM Accel X: %f [m/s^2]\n", pEventLSM_accel->acceleration.x + 0.003);
            printf("LSM Accel Y: %f [m/s^2]\n", pEventLSM_accel->acceleration.y + 0.003);
            printf("LSM Accel Z: %f [m/s^2]\n", pEventLSM_accel->acceleration.z - 9.631);

            printf("LSM Gyro X: %f\n", pEventLSM_gyro->gyro.x + 0.003);
            printf("LSM Gyro Y: %f\n", pEventLSM_gyro->gyro.y+0.115);
            printf("LSM Gyro Z: %f\n", pEventLSM_gyro->gyro.z);

            printf("LSM Temp: %f\n\n", pEventLSM_temp->temperature-2.5);
          #endif
        }
      } else {
          #ifdef DEBUG
              printf("No token LSM!!!");
          #endif
      }

      vTaskDelay(pdMS_TO_TICKS(10)); //Delay for stablity of watchdog

    }
}

void Core1_task(void *pvParameter) {
    TaskParams_t *pTask       = (TaskParams_t *)pvParameter;
    ///GPS pointer
    OutputData_t *pOutputData = pTask->pOutputData;
    ///BNO pointer
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
                #ifdef DEBUG
                        printf("BNO Uptime: %lu [ms]\n", (unsigned long)upTime);

                        printf("BNO Quaternion:\n");
                        printf("BNO quater W: %f\n",   pQuaternion->w());
                        printf("BNO quater X: %f\n",   pQuaternion->x());
                        printf("BNO quater Y: %f\n",   pQuaternion->y());
                        printf("BNO quater Z: %f\n\n", pQuaternion->z());

                        printf("BNO Orientation:\n");
                        printf("BNO Ori X: %f\n",   pEventBNO_ori->orientation.x);
                        printf("BNO Ori Y: %f\n",   pEventBNO_ori->orientation.y);
                        printf("BNO Ori Z: %f\n\n", pEventBNO_ori->orientation.z);

                        printf("BNO Gyro:\n");
                        printf("BNO Gyro X: %f\n",   pEventBNO_angVel->gyro.x);
                        printf("BNO Gyro Y: %f\n",   pEventBNO_angVel->gyro.y);
                        printf("BNO Gyro Z: %f\n\n", pEventBNO_angVel->gyro.z);

                        printf("BNO Magnometer:\n");
                        printf("BNO Mag X: %f\n", pEventBNO_mag->magnetic.x);
                        printf("BNO Mag Y: %f\n", pEventBNO_mag->magnetic.y);
                        printf("BNO Mag Z: %f\n\n", pEventBNO_mag->magnetic.z);

                        printf("BNO Accelerometer:\n");
                        printf("BNO Accel X: %f\n", pEventBNO_accel->acceleration.x);
                        printf("BNO Accel Y: %f\n", pEventBNO_accel->acceleration.y);
                        printf("BNO Accel Z: %f\n\n", pEventBNO_accel->acceleration.z-9.35);
                #endif
            }
        } else {
            #ifdef DEBUG
                printf("No token BNO!!!");
            #endif
        }



        /// GPS function
        bool     got_fix      = false;
        uint32_t cycle_start  = millis();
        uint32_t timeout      = cycle_start + 200;  // ~200 ms per GPS cycle

        while ((millis() < timeout) && !got_fix) {

            if (xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                upTime = xTaskGetTickCount(); 
                // In I2C mode, these all use the bus, so keep them under the lock
                while (GPS.available()) {           // drain all pending bytes this pass
                    GPS.read();                     // reads one byte

                    if (GPS.newNMEAreceived()) {
                        if (!GPS.parse(GPS.lastNMEA())) {
                            // bad sentence, skip it
                            continue;
                        }

                        if (GPS.fix && GPS.satellites > 0) {
                            // Store into shared output struct
                            pOutputData->gps_sats  = GPS.satellites;
                            pOutputData->gps_lat   = GPS.latitude;
                            pOutputData->gps_long  = GPS.longitude;
                            pOutputData->gps_alt   = GPS.altitude;

                            #ifdef DEBUG
                                printf("GPS: fix OK\n");
                                printf("GPS Uptime: %lu [ms]\n", (unsigned long)upTime);
                                printf("Satellites: %d\n", GPS.satellites);
                                printf("Latitude:  %f %c\n", GPS.latitude,  GPS.lat);
                                printf("Longitude: %f %c\n", GPS.longitude, GPS.lon);
                                printf("Altitude:  %f [m]\n\n", GPS.altitude);
                            #endif
                            got_fix = true;
                            break;  // break while(GPS.available())
                        }
                    }
                }

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

        /// SD card saving (Append data) --g get all data for now
        sdData = SD.open("/SD_data.txt", FILE_APPEND);
        if (sdData) {
            sdData.print("\n\n\n\n");
            sdData.print("BMP temp: ");
            sdData.print(gOutputData.bmp_temp);
            sdData.print("\nGPS sattelites: ");
            sdData.print(gOutputData.gps_sats);
            printf("Write success!!!"); 
            sdData.close(); 
        } else {
            printf("Cant open file!!!");
        }

        vTaskDelay(pdMS_TO_TICKS(110)); //REQUIRED DELAY >=110ms (most optimized)

    }
}
