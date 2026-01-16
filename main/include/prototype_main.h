#ifndef PROTOTYPE_MAIN_H
#define PROTOTYPE_MAIN_H

/// Hardware Pins import
#include "Pins.h"

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

void Core0_BMP_task(void *pvParameter);
void Core0_ADXL_task(void *pvParameter);
void Core0_LSM_task(void *pvParameter);
void Core0_stateMachine(void *pvParameter);

void Core1_BNO_task(void *pvParameter);
void Core1_GPS_task(void *pvParameter);
void Core1_SD_task(void *pvParameter);
void Core1_Lora_task(void *pvParameter);

#endif