#include "../include/prototype_main.h"
#include "../constants/Const.h"

void ADXLCalibrate(ADXLCalibrate_t& ADXLCalibrateVars, MagnitudeData_t& MagnitudeData, OutputData_t& OutputData){
    // TODO: [NS/Leads] discuss moving to powerup sequence
    if (!ADXLCalibrateVars.adxl_bias_mean_founded){
        if (ADXLCalibrateVars.adxl_bias_samples_count < ADXL_SAMPLES_MAX){
            ADXLCalibrateVars.adxl_accel_x_mean += OutputData.adxl_acc_x;
            ADXLCalibrateVars.adxl_accel_y_mean += OutputData.adxl_acc_y;
            ADXLCalibrateVars.adxl_accel_z_mean += OutputData.adxl_acc_z;
            ADXLCalibrateVars.adxl_bias_samples_count++;
        } else {
            ADXLCalibrateVars.adxl_accel_x_mean = ADXLCalibrateVars.adxl_accel_x_mean / ADXL_SAMPLES_MAX;
            ADXLCalibrateVars.adxl_accel_y_mean = ADXLCalibrateVars.adxl_accel_y_mean / ADXL_SAMPLES_MAX;
            ADXLCalibrateVars.adxl_accel_z_mean = ADXLCalibrateVars.adxl_accel_z_mean / ADXL_SAMPLES_MAX;
            ADXLCalibrateVars.adxl_bias_mean_founded = true;
        }
    } else {
        OutputData.adxl_acc_x = OutputData.adxl_acc_x - ADXLCalibrateVars.adxl_accel_x_mean;
        OutputData.adxl_acc_y = OutputData.adxl_acc_y - ADXLCalibrateVars.adxl_accel_y_mean;
        OutputData.adxl_acc_z = OutputData.adxl_acc_z - ADXLCalibrateVars.adxl_accel_z_mean;
        // TODO: [NS/Leads] talk about place of this in state machine
        MagnitudeData.adxl_accel_magnitude = sqrtf(OutputData.adxl_acc_x*OutputData.adxl_acc_x + 
                                                   OutputData.adxl_acc_y*OutputData.adxl_acc_y + 
                                                   OutputData.adxl_acc_z*OutputData.adxl_acc_z);
    }
}