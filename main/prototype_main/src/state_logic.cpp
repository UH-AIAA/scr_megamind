#include "../include/prototype_main.h"
#include "../constants/Const.h"


void ADXLIdleToAscend(OutputData_t& gOutputData, MagnitudeData_t& gMagnitudeData, uint8_t& counter, float& peakAltitude) {
    if (gMagnitudeData.adxl_accel_magnitude > ASCEND_THRESHOLD) { 
        counter++; 
        if (counter >= REQ_COUNT_STATE_CHANGE){ 
            gOutputData.flightState = 1; 
            peakAltitude = gOutputData.bmp_alt; 
            counter = 0; 
        } 
    } else {
        counter = 0; 
    }
}

void LSMIdleToAscend(OutputData_t& gOutputData, MagnitudeData_t& gMagnitudeData, uint8_t& counter, float& peakAltitude) {
    if (gMagnitudeData.lsm_accel_magnitude > ASCEND_THRESHOLD) { 
        counter++; 
        if (counter >= REQ_COUNT_STATE_CHANGE) { 
            gOutputData.flightState = 1; 
            peakAltitude = gOutputData.bmp_alt; 
            counter = 0; 
        }
    } else {
        counter = 0; 
    }
}
