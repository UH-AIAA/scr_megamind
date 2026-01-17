#include "../include/prototype_main.h"
#include "../constants/Const.h"


void ADXLIdleToAscend(OutputData_t& gOutputData, MagnitudeData_t& gMagnitudeData, uint8_t& counter, float& peakAltitude) {
    if (gMagnitudeData.adxl_accel_magnitude > ASCEND_THRESHOLD) { /// Check if ADXL magnitude > ascent threshold
        counter++; /// Data is valid -> Increase counter
        if (counter >= REQ_COUNT_STATE_CHANGE){ /// If counter is enough -> valid state change condition
            gOutputData.flightState = 1; /// Change state from IDLE to ASCEND
            peakAltitude = gOutputData.bmp_alt; /// Peak = Current altitude to guarantee peak start correctly on next state
            counter = 0; /// Reset counter to reuse in next state
        } 
    } else {
        counter = 0; /// Reset counter to avoid bad data iterations
    }
}

void LSMIdleToAscend(OutputData_t& gOutputData, MagnitudeData_t& gMagnitudeData, uint8_t& counter, float& peakAltitude) {
    if (gMagnitudeData.lsm_accel_magnitude > ASCEND_THRESHOLD) { /// Check if LSM magnitude > ascent threshold
        counter++; /// Data is valid -> Increase counter
        if (counter >= REQ_COUNT_STATE_CHANGE) { /// If counter is enough -> valid state change condition
            gOutputData.flightState = 1; /// Change state from IDLE to ASCEND
            peakAltitude = gOutputData.bmp_alt; /// Peak = Current altitude to guarantee peak start correctly on next state
            counter = 0; // Reset counter to reuse in next state
        }
    } else {
        counter = 0; /// Reset counter to avoid bad data iterations
    }
}
