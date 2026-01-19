#include "../include/prototype_main.h"
#include "../constants/Const.h"

void IdleToAscend(OutputData_t& gOutputData, MagnitudeData_t& gMagnitudeData, uint8_t& counter, float& peakAltitude) {
    if (gOutputData.adxl_ok) { /// Check if ADXL ok to use
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
    } else if (gOutputData.adxl_ok == false && gOutputData.lsm_ok == true) { /// If ADXL fails and LSM ok to use
        if (gMagnitudeData.lsm_accel_magnitude > ASCEND_THRESHOLD) { 
            counter++; 
            if (counter > REQ_COUNT_STATE_CHANGE + 1) { //skip bad data frame if exists
                gOutputData.flightState = 1; 
                peakAltitude = gOutputData.bmp_alt; 
                counter = 0; 
            }
        } else {
            counter = 0; 
        }
    } else { /// If both ADXL & LSM fail
        counter = 0; /// Reset counter to avoid bad data iterations 
    }
}

void AscendToDescend() {
    
}
