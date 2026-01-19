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

void AscendToDescend(OutputData_t& OutputData, float& bmp_altitude_change, float& bmp_peak_altitude, uint8_t& counter) {
    if (OutputData.bmp_ok) { /// If BMP is ok to use
        if (fabs(bmp_altitude_change) >= BMP_NOISE_THRESHOLD && fabs(bmp_altitude_change) <= BMP_STEP_MAX) { /// If Noise(~0.21) < altitude change < Max possible range means there is actual physical change
            if (OutputData.bmp_alt > bmp_peak_altitude) { /// Check if current altitude > peak
                bmp_peak_altitude = OutputData.bmp_alt; /// Update peak to current altitude
                counter = 0; /// Ensure is counter at 0
            } else {
                float drop_from_peak = bmp_peak_altitude - OutputData.bmp_alt; /// Calculate difference between peak and current data
                if (drop_from_peak > BMP_DESCEND_THRESHOLD) { /// If difference is bigger than dropping threshold
                    counter++; /// Data is valid -> Increase counter
                    if (counter >= REQ_COUNT_STATE_CHANGE) { /// If counter is enough -> valid state change condition *Discuss about counter being higher to reduce risk*
                        OutputData.flightState = 2; /// Change state from ASCEND to DESCEND
                        OutputData.bmp_apogee_record = bmp_peak_altitude; /// Record Apogee to peak
                        counter = 0; /// Reset counter to use in next state
                    }
                } else {
                    counter = 0; /// Means difference is not bigger and data is invalid, then reset counter
                }
            }
        } else {
            counter = 0; /// Means altitude change is invalid, then reset counter and check again
        }  
    }
}
