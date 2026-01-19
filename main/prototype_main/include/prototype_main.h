#ifndef PROTOTYPE_MAIN_H
#define PROTOTYPE_MAIN_H

/// Hardware Pins import
#include "Pins.h"

#include <stdio.h>
#include <bitset>
#include <cmath>

/// @brief 
/*      
        1. Calibrated Data struct
        2. SPI & I2C buses, sensors data and GPS fix validation
        3. Flight State
        4. Apogee
*/
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
      float bmp_apogee_record = 0;
} OutputData_t; 

/// @brief Magnitude data struct
/*
    adxl_accel_magnitude:           Magnitude of ADXL data after calibrated
    lsm_accel_magnitude:            Magnitude of LSM data after calibrated
*/
typedef struct{
      float adxl_accel_magnitude = 0.0; 
      float lsm_accel_magnitude = 0.0; 
} MagnitudeData_t;

/// @brief Helper variables for ADXL calibration
/*
    adxl_accel_x_mean:              Mean of raw ADXL acceleration x data (Bias)
    adxl_accel_y_mean:              Mean of raw ADXL acceleration y data (Bias)
    adxl_accel_z_mean:              Mean of raw ADXL acceleration z data (Bias)
    adxl_bias_samples_count:        Counter to check how many data iteration is sampled
    adxl_bias_mean_founded:         Flag to check if ADXL bias mean is founded
    adxl_accel_magnitude:           Magnitude of ADXL data after calibrated
*/
typedef struct {
    float adxl_accel_x_mean = 0.0;
    float adxl_accel_y_mean = 0.0;
    float adxl_accel_z_mean = 0.0;
    uint8_t adxl_bias_samples_count = 0; 
    bool adxl_bias_mean_founded = false;    
} ADXLCalibrate_t;

/// @brief                   BMP task declaration
/// @param pvParameter       Default FreeRTOS parameter
void Core0_BMP_task(void *pvParameter);

/// @brief                   ADXL task declaration
/// @param pvParameter       Default FreeRTOS parameter
void Core0_ADXL_task(void *pvParameter);

/// @brief                   LSM task declaration
/// @param pvParameter       Default FreeRTOS parameter
void Core0_LSM_task(void *pvParameter);

/// @brief                   State Machine task declaration
/// @param pvParameter       Default FreeRTOS parameter
void Core0_stateMachine(void *pvParameter);

/// @brief                   BNO task declaration
/// @param pvParameter       Default FreeRTOS parameter
void Core1_BNO_task(void *pvParameter);

/// @brief GPS task declaration
/// @param pvParameter default FreeRTOS parameter
void Core1_GPS_task(void *pvParameter);

/// @brief                   SD task declaration
/// @param pvParameter       Default FreeRTOS parameter
void Core1_SD_task(void *pvParameter);

/// @brief                   Lora task declaration
/// @param pvParameter       Default FreeRTOS parameter
void Core1_Lora_task(void *pvParameter);

/// @brief                   1. Find bias of ADXL acceleration in x,y,z
///                          2. Calibrate data
///                          3. Calculate magnitude
/// @param ADXLCalibrateVars ADXL helper variables struct
/// @param MagnitudeData     Magnitude data struct
/// @param OutputData        General data struct
void ADXLCalibrate(ADXLCalibrate_t& ADXLCalibrateVars, MagnitudeData_t& MagnitudeData, OutputData_t& OutputData);

/// @brief                   1. Check ADXL magnitude against ascent threshold
///                          2. Data is valid -> Increase counter
///                          3. If counter suffices -> valid state change condition ->  Change state from IDLE to ASCEND -> Record current peak -> reset counter for next state
///                             If counter not suffices -> Reset counter
/// @param OutputData        General data struct
/// @param MagnitudeData     Magnitude data struct
/// @param counter           State change counter 
/// @param peakAltitude      Peak altitude variable
void ADXLIdleToAscend(OutputData_t& OutputData, MagnitudeData_t& MagnitudeData, uint8_t& counter, float& peakAltitude);

/// @brief                   1. Check LSM magnitude against ascent threshold
///                          2. Data is valid -> Increase counter
///                          3. If counter suffices -> valid state change condition ->  Change state from IDLE to ASCEND -> Record current peak -> reset counter for next state
///                             If counter not suffices -> Reset counter
/// @param OutputData        General data struct
/// @param MagnitudeData     Magnitude data struct
/// @param counter           State change counter 
/// @param peakAltitude      Peak altitude variable
void LSMIdleToAscend(OutputData_t& OutputData, MagnitudeData_t& MagnitudeData, uint8_t& counter, float& peakAltitude);

void IdleToAscend(OutputData_t& gOutputData, MagnitudeData_t& gMagnitudeData, uint8_t& counter, float& peakAltitude);

#endif