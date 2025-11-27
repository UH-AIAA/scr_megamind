#include <stdio.h>
#include <bitset>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "driver/gpio.h"

/// SPI imports, I^2C, and UART Imports
#include "SPI.h"
#include "Arduino.h"

/// Library Import
#include "Adafruit_BMP5xx.h"
#include "Adafruit_ADXL375.h"
#include "Adafruit_LSM6DSO32.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_GPS.h"
#include "LoRa.h"
#include "SD.h"

/// Hardware Pins import
#include "Pins.h"

/// Define frequency Lora + SD
#define LORA_FREQ 915E6 /// 915MHz frequency for Lora
#define SD_RATE 20E6    /// 20MHz rate for SD 

/// Debug control 
#define DEBUG

/// @brief Calibrated Data struct
///        SPI & I2C buses, sensors data and GPS fix
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
    uint8_t gps_sats = 0;
    float gps_lat, gps_long, gps_alt = 0;
    
    bool spi1_ok = false;
    bool spi2_ok = false;
    bool i2c_ok = false;
    bool bmp_ok = false;
    bool adxl_ok = false;
    bool lsm_ok = false;
    bool bno_ok = false;
    bool gpsFix_ok = false;
    bool sd_ok = false;
    bool lora_ok = false;

    uint8_t flightState = 0;

} OutputData_t; 

/*
@brief: Object for sensors data retrieving
    gOutputData:                   global Output object for all sensors
    gEventADXL :                   global output data for ADXL375
    gEventLSM_ :                   global output data for LSM 
    gOrientation to gAcceleromter: global output for BNO
    gQuarternion:                  quaternion output for BNO
*/
OutputData_t    gOutputData; 
sensors_event_t gEventADXL; 
sensors_event_t gEventLSM_accel, gEventLSM_gyro, gEventLSM_temp; 
sensors_event_t gOrientation, gAngVelocity, gMagnetometer, gAccelerometer; 
imu::Quaternion gQuaternion; 

/*
@brief: sensors helper variables
    upTime:                       Current Output time [ms] of each sensor
    gSpiMutex_BAL:                SPI mutex for Bmp + Adxl + Lsm
    gSpiMutex_SL:                 SPI mutex for Sd + Lora
    gI2cMutex :                   I2C mutex for Bno + Gps
    gHasSD:                       Check for micro SD card
    MAX_GPS_BYTES_PER_LOOP:       Capped amount of bytes permitted when reading GPS data (tune as needed)
    loraCounter:                  Counter to count data packets sended
*/
uint32_t upTime;             
SemaphoreHandle_t gSpiMutex_BAL; 
SemaphoreHandle_t gSpiMutex_SL; 
SemaphoreHandle_t gI2cMutex;    
bool gHasSD = false;

// TODO: [NS] does this need to be static?
static uint16_t loraCounter = 0;
const int MAX_GPS_BYTES_PER_LOOP = 64;  

/*
@brief: Constant for state machine
    ACCEL_LAUNCH_G:               G force multiplier for state change detection
    GRAVITY_FORCE:                gravity force of the Earth
    REQ_COUNT_STATE_CHANGE:       Amount of check before state is allowed to change
    ASCEND_THRESHOLD:             Ascent threshold    
*/
// TODO: [NS] think about making these #define
const int ACCEL_LAUNCH_G = 2; 
const float GRAVITY_FORCE = 9.80665;
const int REQ_COUNT_STATE_CHANGE = 3;
const float ASCEND_THRESHOLD = GRAVITY_FORCE * ACCEL_LAUNCH_G;
const float BMP_STANDARD_DEVIATION = 0.07594;
const int BMP_NOISE_MULTIPLIER = 3;
const float BMP_ALTITUDE_CHANGE_THRESHOLD = BMP_STANDARD_DEVIATION * BMP_NOISE_MULTIPLIER;

/*
@brief: Helper variables for state machine
    counter_state_change:           Counter for State change
    bmp_previous_altitude:          Hold previous altitude value of BMP
    bmp_previous_altitude_founded:  Flag to check if previous BMP altitude exists
*/
// TODO: [NS] does this also need to be static?
static int counter_state_change = 0;
static float bmp_previous_altitude = 0;
static bool bmp_previous_altitude_founded = false;


/*
@brief: helper variables for ADXL calibration
    ADXL_SAMPLES_MAX:               Maximum data samples for calibration
    adxl_accel_x_mean:              Mean of raw ADXL acceleration x data (Bias)
    adxl_accel_y_mean:              Mean of raw ADXL acceleration y data (Bias)
    adxl_accel_z_mean:              Mean of raw ADXL acceleration z data (Bias)
    adxl_bias_samples_count:        Counter to check how many data iteration is sampled
    adxl_bias_mean_founded:         Flag to check if ADXL bias mean is founded
    adxl_accel_magnitude:           Magnitude of ADXL data after calibrated
*/
// TODO: [NS] same comment about static
const int ADXL_SAMPLES_MAX = 20; 
static float adxl_accel_x_mean = 0.0;
static float adxl_accel_y_mean = 0.0;
static float adxl_accel_z_mean = 0.0;
static uint8_t adxl_bias_samples_count = 0; 
static bool adxl_bias_mean_founded = false; 
static float adxl_accel_magnitude = 0.0;    

/*
@brief: helper variables for LSM calibration
    lsm_samples_max:               Maximum data samples for calibration
    lsm_accel_x_mean:              Mean of raw LSM acceleration x data (Bias)
    lsm_accel_y_mean:              Mean of raw LSM acceleration y data (Bias)
    lsm_accel_z_mean:              Mean of raw LSM acceleration z data (Bias)
    lsm_bias_samples_count:        Counter to check how many data iteration is sampled
    lsm_bias_mean_founded:         Flag to check if LSM bias mean is founded
    lsm_accel_magnitude:           Magnitude of LSM data after calibrated
*/
// TODO: [NS] same comment about static
const int LSM_SAMPLES_MAX = 20; 
static float lsm_accel_x_mean = 0.0;
static float lsm_accel_y_mean = 0.0;
static float lsm_accel_z_mean = 0.0;
static uint8_t lsm_bias_samples_count = 0; 
static bool lsm_bias_mean_founded = false;
static float lsm_accel_magnitude = 0.0;    

/*
@brief: helper variables for BMP calibration
    BMP_SAMPLES_MAX:               Maximum data samples for calibration
    bmp_altitude_mean:             Mean of raw BMP acceleration x data (Bias)
    bmp_bias_samples_count:        Counter to check how many data iteration is sampled
    bmp_bias_mean_founded:         Flag to check if BMP bias mean is founded
*/
// TODO: [NS] same comment about static
const int BMP_SAMPLES_MAX = 20;
static float bmp_altitude_mean = 0.0;
static uint8_t bmp_bias_samples_count = 0; 
static bool bmp_bias_mean_founded = false; 

/// Sensors Object Instantiation
Adafruit_BMP5xx     BMP;
Adafruit_ADXL375    ADXL(ADXL375_CS, &SPI);
Adafruit_LSM6DSO32  LSM;
Adafruit_BNO055     BNO(55, BNO055_ADDRESS_A, &Wire);
Adafruit_GPS        GPS(&Wire);
SPIClass            SPI2(HSPI);
File                sdData;


void init_spi() {
    /// Initialize SPI bus for BMP + ADXL + LSM
    gOutputData.spi1_ok = SPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1);
    /// Initialize BMP
    BMP.begin(BMP581_CS, &SPI);
    /// Initialize ADXL
    ADXL.begin();
    /// Initialize LSM
    LSM.begin_SPI(LSM6DSO32_CS, &SPI);
    // TODO: [MEMBERS]: add LSM data rate config

    /// Initialize SPI bus for SD + Lora
    gOutputData.spi2_ok = SPI2.begin(VSPI_SCLK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN, -1);
    /// Initialize SD
    gHasSD = SD.begin(SD_CS, SPI2, SD_RATE);
    /// Set pins & Initialize Lora to defined frequency
    LoRa.setSPI(SPI2);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_G0_INT);
    LoRa.begin(LORA_FREQ);
}

void init_I2C() {
    /// Initialize I2C bus for BNO + GPS
    gOutputData.i2c_ok = Wire.begin(I2C_SDA, I2C_SCL);
    /// Initialize BNO
    BNO.begin();
}

void Core0_BMP_task(void *pvParameter);
void Core0_ADXL_task(void *pvParameter);
void Core0_LSM_task(void *pvParameter);
void Core0_stateMachine(void *pvParameter);

void Core1_BNO_task(void *pvParameter);
void Core1_GPS_task(void *pvParameter);
void Core1_SD_task(void *pvParameter);
void Core1_Lora_task(void *pvParameter);

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
    gSpiMutex_BAL = xSemaphoreCreateMutex();
    gSpiMutex_SL  = xSemaphoreCreateMutex();
    gI2cMutex     = xSemaphoreCreateMutex();
    /// Tasks to cores
    /// Core 0
    xTaskCreatePinnedToCore(Core0_BMP_task, "BMP_task", 5000, NULL,
                            3, NULL, 0);
    xTaskCreatePinnedToCore(Core0_ADXL_task, "ADXL_task", 5000, NULL,
                            4, NULL, 0); 
    xTaskCreatePinnedToCore(Core0_LSM_task, "LSM_task", 5000, NULL,
                            2, NULL, 0);   
    xTaskCreatePinnedToCore(Core0_stateMachine, "Core0_stateMachine", 5000, NULL,
                            1, NULL, 0);                  
    /// Core 1
    xTaskCreatePinnedToCore(Core1_BNO_task, "Core1_BNO_task", 5000, NULL,
                            1, NULL, 1);
    xTaskCreatePinnedToCore(Core1_GPS_task, "Core1_GPS_task", 5000, NULL,
                            2, NULL, 1);
    xTaskCreatePinnedToCore(Core1_SD_task, "Core1_SD_task", 5000, NULL,
                            4, NULL, 1);
    xTaskCreatePinnedToCore(Core1_Lora_task, "Core1_Lora_task", 5000, NULL,
                            3, NULL, 1);
}

void Core0_BMP_task(void *pvParameter) {
    while (1) {
        /// BMP functions + calibrations
        if (xSemaphoreTake(gSpiMutex_BAL, pdMS_TO_TICKS(10)) == pdTRUE) {
            upTime = xTaskGetTickCount();
            gOutputData.bmp_ok = BMP.performReading();
            
            xSemaphoreGive(gSpiMutex_BAL);

            if (gOutputData.bmp_ok) {
                // calibrate here & save to data struct 
                gOutputData.bmp_temp  = BMP.temperature;
                gOutputData.bmp_press = BMP.pressure;
                gOutputData.bmp_alt   = BMP.readAltitude(1013.25f);

                if (!bmp_bias_mean_founded){
                    if (bmp_bias_samples_count < BMP_SAMPLES_MAX){
                        bmp_altitude_mean += gOutputData.bmp_alt;
                        bmp_bias_samples_count++;
                    } else {
                        bmp_altitude_mean = bmp_altitude_mean / BMP_SAMPLES_MAX;
                        bmp_bias_mean_founded = true;
                    }
                } else {
                    gOutputData.bmp_alt = fabs(gOutputData.bmp_alt - bmp_altitude_mean);
                }

            #ifdef DEBUG
                printf("BMP Read Time: %lu [ms]\n", (unsigned long)upTime);
                printf("BMP Temp: %f\n", gOutputData.bmp_temp);
                printf("BMP Press: %f\n", gOutputData.bmp_press);
                printf("BMP Alt: %f\n\n", gOutputData.bmp_alt);
            #endif
            }
        } else {
            #ifdef DEBUG
                printf("No token BMP!!!\n");
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(15)); //Stable, Unoptimized
    }
}

void Core0_ADXL_task(void *pvParameter) {

    while (1) {
        /// ADXL function + calibrations
        if (xSemaphoreTake(gSpiMutex_BAL, pdMS_TO_TICKS(10)) == pdTRUE) {
            upTime = xTaskGetTickCount();
            gOutputData.adxl_ok = ADXL.getEvent(&gEventADXL);
            xSemaphoreGive(gSpiMutex_BAL);

            if (gOutputData.adxl_ok) {
                // calibrate here & save to data struct 
                gOutputData.adxl_acc_x = gEventADXL.acceleration.x;
                gOutputData.adxl_acc_y = gEventADXL.acceleration.y;
                gOutputData.adxl_acc_z = gEventADXL.acceleration.z;

                // TODO: [NS/Leads] discuss moving to powerup sequence
                if (!adxl_bias_mean_founded){
                    if (adxl_bias_samples_count < ADXL_SAMPLES_MAX){
                        adxl_accel_x_mean += gOutputData.adxl_acc_x;
                        adxl_accel_y_mean += gOutputData.adxl_acc_y;
                        adxl_accel_z_mean += gOutputData.adxl_acc_z;
                        adxl_bias_samples_count++;
                    } else {
                        adxl_accel_x_mean = adxl_accel_x_mean / ADXL_SAMPLES_MAX;
                        adxl_accel_y_mean = adxl_accel_y_mean / ADXL_SAMPLES_MAX;
                        adxl_accel_z_mean = adxl_accel_z_mean / ADXL_SAMPLES_MAX;
                        adxl_bias_mean_founded = true;
                    }
                } else {
                    gOutputData.adxl_acc_x = gOutputData.adxl_acc_x - adxl_accel_x_mean;
                    gOutputData.adxl_acc_y = gOutputData.adxl_acc_y - adxl_accel_y_mean;
                    gOutputData.adxl_acc_z = gOutputData.adxl_acc_z - adxl_accel_z_mean;
                    // TODO: [NS/Leads] talk about place of this in state machine
                    adxl_accel_magnitude = sqrtf(gOutputData.adxl_acc_x*gOutputData.adxl_acc_x + 
                                                 gOutputData.adxl_acc_y*gOutputData.adxl_acc_y + 
                                                 gOutputData.adxl_acc_z*gOutputData.adxl_acc_z);
                }

                #ifdef DEBUG
                        printf("ADXL Up Time: %lu [ms]\n", (unsigned long)upTime);
                        printf("ADXL accel X: %f [m/s^2]\n", gOutputData.adxl_acc_x);
                        printf("ADXL accel Y: %f [m/s^2]\n", gOutputData.adxl_acc_y);
                        printf("ADXL accel Z: %f [m/s^2]\n\n", gOutputData.adxl_acc_z);

                        printf("ADXL ACCELERATION MAGNITUDE: %f [m/s^2]\n\n", adxl_accel_magnitude);
                #endif
            }
        } else {
            #ifdef DEBUG
                printf("No token ADXL!!!\n");
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(20)); //Stable, Unoptimized
    }
}

void Core0_LSM_task(void *pvParameter) {

    while (1) {
        /// LSM function + calibrations
        if (xSemaphoreTake(gSpiMutex_BAL, pdMS_TO_TICKS(10)) == pdTRUE) {
            upTime = xTaskGetTickCount();
            gOutputData.lsm_ok = LSM.getEvent(&gEventLSM_accel, &gEventLSM_gyro, &gEventLSM_temp);
            xSemaphoreGive(gSpiMutex_BAL);

            if (gOutputData.lsm_ok) {
                // calibrate here & save to data struct 
                gOutputData.lsm_acc_x  = gEventLSM_accel.acceleration.x;
                gOutputData.lsm_acc_y  = gEventLSM_accel.acceleration.y;
                gOutputData.lsm_acc_z  = gEventLSM_accel.acceleration.z;
                gOutputData.lsm_gyro_x = gEventLSM_gyro.gyro.x;
                gOutputData.lsm_gyro_y = gEventLSM_gyro.gyro.y;
                gOutputData.lsm_gyro_z = gEventLSM_gyro.gyro.z;
                gOutputData.lsm_temp   = gEventLSM_temp.temperature;
                

                if (!lsm_bias_mean_founded){
                    if (lsm_bias_samples_count < LSM_SAMPLES_MAX){
                        lsm_accel_x_mean += gOutputData.lsm_acc_x;
                        lsm_accel_y_mean += gOutputData.lsm_acc_y;
                        lsm_accel_z_mean += gOutputData.lsm_acc_z;
                        lsm_bias_samples_count++;
                    } else {
                        lsm_accel_x_mean = lsm_accel_x_mean / LSM_SAMPLES_MAX;
                        lsm_accel_y_mean = lsm_accel_y_mean / LSM_SAMPLES_MAX;
                        lsm_accel_z_mean = lsm_accel_z_mean / LSM_SAMPLES_MAX;
                        lsm_bias_mean_founded = true;
                    }
                } else {
                    gOutputData.lsm_acc_x = gOutputData.lsm_acc_x - lsm_accel_x_mean;
                    gOutputData.lsm_acc_y = gOutputData.lsm_acc_y - lsm_accel_y_mean;
                    gOutputData.lsm_acc_z = gOutputData.lsm_acc_z - lsm_accel_z_mean;
                    lsm_accel_magnitude = sqrtf(gOutputData.lsm_acc_x*gOutputData.lsm_acc_x + 
                                                gOutputData.lsm_acc_y*gOutputData.lsm_acc_y + 
                                                gOutputData.lsm_acc_z*gOutputData.lsm_acc_z);
                }

                

                #ifdef DEBUG
                    printf("LSM Up Time: %lu [ms]\n", (unsigned long)upTime);
                    printf("LSM Accel X: %f [m/s^2]\n", gOutputData.lsm_acc_x);
                    printf("LSM Accel Y: %f [m/s^2]\n", gOutputData.lsm_acc_y);
                    printf("LSM Accel Z: %f [m/s^2]\n", gOutputData.lsm_acc_z);

                    printf("LSM Gyro X: %f\n", gOutputData.lsm_gyro_x);
                    printf("LSM Gyro Y: %f\n", gOutputData.lsm_gyro_y);
                    printf("LSM Gyro Z: %f\n", gOutputData.lsm_gyro_z);

                    printf("LSM Temp: %f\n\n", gOutputData.lsm_temp);

                    printf("LSM ACCELERATION MAGNITUDE: %f\n\n", lsm_accel_magnitude);
                #endif
            }
        } else {
            #ifdef DEBUG
                printf("No token LSM!!!\n");
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(15)); //Stable, Unoptimized
    }
}

void Core0_stateMachine(void *pvParameter){
    while (1) {
        switch (gOutputData.flightState){
        case 0: /// IDLE 
            #ifdef DEBUG
                printf("STATE IDLE--------------------------\n");
            #endif
            if (gOutputData.adxl_ok) { // If ADXL ok
                if (adxl_accel_magnitude > ASCEND_THRESHOLD) { // Check if ADXL magnitude > then ascent threshold
                    counter_state_change++;
                    if (counter_state_change == REQ_COUNT_STATE_CHANGE){
                        gOutputData.flightState = 1; // Change state from IDLE to ASCEND
                        counter_state_change = 0; // Reset counter to reuse in another state
                    }
                }
            } else if (gOutputData.lsm_ok){ // If ADXL fails and LSM ok
                if (lsm_accel_magnitude > ASCEND_THRESHOLD) { // Check if LSM magnitude > then ascent threshold
                    counter_state_change++;
                    if (counter_state_change == REQ_COUNT_STATE_CHANGE){
                        gOutputData.flightState = 1; // Change state from IDLE to ASCEND
                        counter_state_change = 0; // reset counter to reuse in another state
                    }
                }
            }
            // TODO: [NS/leads] talk about a counter reset condition
            break;
        case 1: /// ASCEND
            #ifdef DEBUG
                printf("STATE ASCENDING---------------------\n");
            #endif
            
            break;
        case 2: /// DESCEND
            #ifdef DEBUG
                printf("STATE DESCENDING--------------------------\n");
            #endif
            /// TBD
            break;
        case 3: /// LANDED
            #ifdef DEBUG
                printf("STATE LANDED--------------------------\n");
            #endif
            /// TBD
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void Core1_BNO_task(void *pvParameter) {
    while (1) {
        ///BNO function 
        // Leaving printout for delay optimization later
        bool orient_ok = false;
        bool gyro_ok   = false;
        bool mag_ok    = false;
        bool accel_ok  = false;

        if (xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            upTime = xTaskGetTickCount();
            orient_ok = BNO.getEvent(&gOrientation,    Adafruit_BNO055::VECTOR_EULER);
            gyro_ok   = BNO.getEvent(&gAngVelocity, Adafruit_BNO055::VECTOR_GYROSCOPE);
            mag_ok    = BNO.getEvent(&gMagnetometer,    Adafruit_BNO055::VECTOR_MAGNETOMETER);
            accel_ok  = BNO.getEvent(&gAccelerometer,  Adafruit_BNO055::VECTOR_ACCELEROMETER);
            // quaternion also uses I2C, so keep it inside the lock
            gQuaternion = BNO.getQuat();
            xSemaphoreGive(gI2cMutex);

            gOutputData.bno_ok = orient_ok && gyro_ok && mag_ok && accel_ok;
            if (gOutputData.bno_ok) {
                /// calibrate here & save to data struct 
                gOutputData.bno_quar_w = gQuaternion.w();
                gOutputData.bno_quar_x = gQuaternion.x();
                gOutputData.bno_quar_y = gQuaternion.y();
                gOutputData.bno_quar_z = gQuaternion.z();

                gOutputData.bno_ori_x  = gOrientation.orientation.x;
                gOutputData.bno_ori_y  = gOrientation.orientation.y;
                gOutputData.bno_ori_z  = gOrientation.orientation.z;

                gOutputData.bno_gyro_x = gAngVelocity.gyro.x;
                gOutputData.bno_gyro_y = gAngVelocity.gyro.y;
                gOutputData.bno_gyro_z = gAngVelocity.gyro.z;

                gOutputData.bno_mag_x  = gMagnetometer.magnetic.x;
                gOutputData.bno_mag_y  = gMagnetometer.magnetic.y;
                gOutputData.bno_mag_z  = gMagnetometer.magnetic.z;

                gOutputData.bno_acc_x  = gAccelerometer.acceleration.x;
                gOutputData.bno_acc_y  = gAccelerometer.acceleration.y;
                gOutputData.bno_acc_z  = gAccelerometer.acceleration.z;

                #ifdef DEBUG
                    printf("BNO Uptime: %lu [ms]\n", (unsigned long)upTime);

                    printf("BNO Quaternion:\n");
                    printf("BNO quater W: %f\n",   gOutputData.bno_quar_w);
                    printf("BNO quater X: %f\n",   gOutputData.bno_quar_x);
                    printf("BNO quater Y: %f\n",   gOutputData.bno_quar_y);
                    printf("BNO quater Z: %f\n\n", gOutputData.bno_quar_z);

                    printf("BNO Orientation:\n");
                    printf("BNO Ori X: %f\n",   gOutputData.bno_ori_x);
                    printf("BNO Ori Y: %f\n",   gOutputData.bno_ori_y);
                    printf("BNO Ori Z: %f\n\n", gOutputData.bno_ori_z);

                    printf("BNO Gyro:\n");
                    printf("BNO Gyro X: %f\n",   gOutputData.bno_gyro_x);
                    printf("BNO Gyro Y: %f\n",   gOutputData.bno_gyro_y);
                    printf("BNO Gyro Z: %f\n\n", gOutputData.bno_gyro_z);

                    printf("BNO Magnometer:\n");
                    printf("BNO Mag X: %f\n", gOutputData.bno_mag_x);
                    printf("BNO Mag Y: %f\n", gOutputData.bno_mag_y);
                    printf("BNO Mag Z: %f\n\n", gOutputData.bno_mag_z);

                    printf("BNO Accelerometer:\n");
                    printf("BNO Accel X: %f\n", gOutputData.bno_acc_x);
                    printf("BNO Accel Y: %f\n", gOutputData.bno_acc_y);
                    printf("BNO Accel Z: %f\n\n", gOutputData.bno_acc_z);
                #endif

            }
        } else {

            #ifdef DEBUG
                printf("No token BNO!!!\n");
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(300)); //Stable, Unoptimized
    }
}

// TODO: [NS] look at safety structures in this function
void Core1_GPS_task(void *pvParameter) {
    while (1) {
        /// GPS function 
        // Leaving print out to help optmize delay and timing later
        int bytes_processed = 0;
        bool     got_fix     = false;
        uint32_t cycle_start = millis();
        // TODO: [NS] make 200ms a #define constant
        uint32_t timeout     = cycle_start + 200;  // ~200 ms window for this cycle

        while ((millis() < timeout) && !got_fix) { // Might be redudant since data is capped (Stable - Unoptimized)
            // Try to grab I2C bus briefly
            if (xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                upTime = xTaskGetTickCount();
                // Drain only certain bytes currently in the GPS buffer to PREVENT INFINITE LOOP (Stable-Unoptimized)
                while (GPS.available() && bytes_processed < MAX_GPS_BYTES_PER_LOOP) {
                    GPS.read();  // reads ONE byte
                    if (GPS.newNMEAreceived()) {
                        // Parse only good sentences
                        if (!GPS.parse(GPS.lastNMEA())) {
                            continue;  // bad sentence, skip
                        }
                        // Check for a valid fix with satellites
                        if (GPS.fix && GPS.satellites > 0) {
                            // Save into shared struct
                            gOutputData.gps_sats = GPS.satellites;
                            gOutputData.gps_lat  = GPS.latitude;
                            gOutputData.gps_long = GPS.longitude;
                            gOutputData.gps_alt  = GPS.altitude;

                            #ifdef DEBUG
                                printf("GPS: fix OK\n");
                                printf("GPS Uptime: %lu [ticks]\n", (unsigned long)upTime);
                                printf("Satellites: %d\n", gOutputData.gps_sats);
                                printf("Latitude:  %f %c\n", gOutputData.gps_lat,  GPS.lat);
                                printf("Longitude: %f %c\n", gOutputData.gps_long, GPS.lon);
                                printf("Altitude:  %f [m]\n\n", gOutputData.gps_alt);
                            #endif

                            got_fix = true;
                            gOutputData.gpsFix_ok = true;
                            break;  // break while(GPS.available())
                        }
                    }
                    bytes_processed++;
                }
                xSemaphoreGive(gI2cMutex);
            } else {
                #ifdef DEBUG
                    printf("No token GPS!!!\n");
                #endif
            }
        }

        if (!got_fix) {
            #ifdef DEBUG
                printf("GPS: no valid fix this cycle\n");
            #endif
            gOutputData.gpsFix_ok = false;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Stable, Unoptimized
    }
}

void Core1_SD_task(void *pvParameter) {
    while (1) {
        /// SD card saving (Append data) - get all data
        /// Only txt works. csv triggers watchdog
        if (xSemaphoreTake(gSpiMutex_SL, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (gHasSD){
                sdData = SD.open("/SD_data.txt", FILE_APPEND);
                gOutputData.sd_ok = sdData;
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
                                    "gps_sats,gps_lat,gps_long,gps_alt,"
                                    "Flight State"
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
                    sdData.print(gOutputData.gps_alt);    sdData.print(',');
                    sdData.print(gOutputData.flightState);

                    sdData.println(); 

                    #ifdef DEBUG
                        printf("Write Success!!\n");
                    #endif

                    sdData.flush();
                    xSemaphoreGive(gSpiMutex_SL);
                } else {
                    xSemaphoreGive(gSpiMutex_SL);
                    
                    #ifdef DEBUG
                        printf("Cant open file!!!\n");
                    #endif

                }
            } else {
                xSemaphoreGive(gSpiMutex_SL);

                #ifdef DEBUG
                    printf("No SD card found!!!\n");
                #endif
                
            }
        } else {

            #ifdef DEBUG
                printf("No Token SD");
            #endif

        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Stable, Unoptimized
    }
}

void Core1_Lora_task(void *pvParameter) {
    while (1) {
        /// Lora function
        if (xSemaphoreTake(gSpiMutex_SL, pdMS_TO_TICKS(10)) == pdTRUE) {
            LoRa.beginPacket();
            gOutputData.lora_ok = true;
            /// Packet Counter
            LoRa.write((uint8_t*)&loraCounter, sizeof(loraCounter));
            /// Send telemetry payload
            LoRa.write((uint8_t*)&gOutputData, sizeof(gOutputData));
            LoRa.endPacket();
            loraCounter++;
            xSemaphoreGive(gSpiMutex_SL);
        }
        else {
            
            #ifdef DEBUG
                printf("No Token Lora!!!!!");
            #endif

        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Stable, Unoptimized
    }
}