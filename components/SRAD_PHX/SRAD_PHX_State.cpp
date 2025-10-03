/* SRAD Avionics Flight Software for AIAA-UH
 *
 * Copyright (c) 2024 Nathan Samuell + Dedah + Thanh! (www.github.com/nathansamuell, www.github.com/UH-AIAA)
 *
 * More information on the MIT license as well as a complete copy
 * of the license can be found here: https://choosealicense.com/licenses/mit/
 *
 * All above text must be included in any redistribution.
 */

#include "SRAD_PHX.h"


/**
 * @brief gets flight state
 * 
 * The function uses a cascading switch case to determine which stage
 * of flight the rocket is in. At each stage, it calls a helper function
 * to determine if it should move to the next one.
 */
void FLIGHT::calculateState() {
    switch(STATE) {
        case(STATES::PRE_NO_CAL):
            AltitudeCalibrate(); //check altitude offset and set it
            if(calibrate()) {
                STATE = STATES::PRE_CAL;
            }
            break;

        case(STATES::PRE_CAL):
            AltitudeCalibrate(); //check altitude offset and set it
            if(isAscent()) {
                STATE = STATES::FLIGHT_ASCENT;
            }
            break;

        case STATES::FLIGHT_ASCENT:
            if(isDescent()) {
                STATE = STATES::FLIGHT_DESCENT;
            }
            break;

        case STATES::FLIGHT_DESCENT:
            if(isLanded()) {
                STATE = STATES::POST_LANDED;
            }
            break;
        case STATES::POST_LANDED:
            break;
    }
}
/**
 * Helper function to check if sensors are calibrated
 * @return returns true if sensors are calibrated
 */
bool FLIGHT::isCal() {
    return calibrated;
}

/**
 * Helper function to check if rocket is ascending
 * Fault tolerant for failure of LSM or ADXL.
 * 1. If LSM fails, default to ADXL
 * 2. If ADXL fails, default to BMP
 * @return returns true if rocket is ascending
 */
bool FLIGHT::isAscent() {
    static uint32_t liftoffTimer_ms;
    if(data.sensor_status[0] == 1) {
        if(data.lsm_acc_z > accel_liftoff_threshold) {
            liftoffTimer_ms += deltaTime_ms;

            if(liftoffTimer_ms  > accel_liftoff_time_threshold) {
                return true;
            }
        } else {
            liftoffTimer_ms = 0;
        }
    } else if (data.sensor_status[2] == 1) {  // if primary accel is known to be bad, check secondary
        if(data.lsm_acc_z > accel_liftoff_threshold) {
            liftoffTimer_ms += deltaTime_ms;

            if(liftoffTimer_ms  > accel_liftoff_time_threshold) {
                return true;
            }
        } 
        else {
            liftoffTimer_ms = 0;
        }
    } else {  // if both accelerometers are bad, use altimeter
        // check if altitude is notably higher than 0 (or alt threshold)
    }
    return false;
}

bool FLIGHT::isDescent() {
    // use altimeter primarily to detect apogee based off of trend in data
    if(data.sensor_status[1] == 1) { // TODO: fix whole if bock to use uint8
        uint8_t desc_samples = 0;                                   // tracks the number of samples with a descending delta
        Serial.println("made it to for loop");
        for(int i = 0; i < 9; i++) {
            Serial.print("loop iteration: "); Serial.println(i);
            uint8_t index1 = (altReadings_ind + i + 1) % 10;        // get an index, starting with our oldest value
            uint8_t index2 = (altReadings_ind + i + 2) % 10;        // get the value after the first index
            if(altReadings[index1] - altReadings[index2] > 0.5 ) {  // change 0.5 based on noise values from lab testing
                desc_samples++;
            }
        }
        if(desc_samples > 7) {
            return true;
        }
    } // add backup sensor here

    return false;
}

bool FLIGHT::isLanded() {
    if(data.sensor_status[0] == 1) {
        if (data.adxl_acc_z < 2 && data.adxl_acc_z >= 0){
            return true;
        }
    } else {
        if (data.bmp_alt <=  alt_offset + 10){    // if the current altitude is less than the offset altitude + 10m 
            return true;                            // then return True to indicate the rocket is landed
        }
    }

    return false;
}

bool FLIGHT::calibrate() {
    // calibrate for GPS offset, possibly of the earth spinning?
    //
    // additionally calibrate altitude offset

    return false;
}

void FLIGHT::AltitudeCalibrate() {
    // save the offset to the current altitude when the function is called
    alt_offset = data.bmp_alt;
}

