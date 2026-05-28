// Nathan Samuell C 2026
#include "megamind.h"

// returns true on success, false on failure, TODO: [NS] add error modes to this refactor
bool ReadADXL(Adafruit_ADXL375* ADXL, GDQMessage_t *outputMsg, float accelBias[3])
{
    static sensors_event_t event;
    // if read operation fails, start task over:
    if(!ADXL->getEvent(&event)) {
        // xSemaphoreGive(sensor_spi_mutex);
        return false;
    }

    // save off our sensor data, add it to queue
    outputMsg->ADXLMessage.acceleration[0] = float(event.acceleration.x) - accelBias[0];
    outputMsg->ADXLMessage.acceleration[1] = float(event.acceleration.y) - accelBias[1];
    outputMsg->ADXLMessage.acceleration[2] = float(event.acceleration.z) - accelBias[2];

    return true;
}

bool ReadBNO(Adafruit_BNO055 *BNO, GDQMessage_t *currentMessage)
{
    sensors_event_t orientationData, angVelocityData, magnetometerData, accelerometerData;
    if (!BNO->getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
        return false;
    }
    if (!BNO->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
        return false;
    }
    if (!BNO->getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER)) {
        return false;
    }
    if (!BNO->getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER)) {
        return false;
    }

    imu::Quaternion quat = BNO->getQuat();

    //save sensor data and add it to queue
    currentMessage->BNOMessage.quaternion[0] = quat.w();
    currentMessage->BNOMessage.quaternion[1] = quat.x();
    currentMessage->BNOMessage.quaternion[2] = quat.y();
    currentMessage->BNOMessage.quaternion[3] = quat.z();

    currentMessage->BNOMessage.euler_orientation[0] = angVelocityData.gyro.x;
    currentMessage->BNOMessage.euler_orientation[1] = angVelocityData.gyro.y;
    currentMessage->BNOMessage.euler_orientation[2] = angVelocityData.gyro.z;

    currentMessage->BNOMessage.magnetometer[0] = magnetometerData.magnetic.x;
    currentMessage->BNOMessage.magnetometer[1] = magnetometerData.magnetic.y;
    currentMessage->BNOMessage.magnetometer[2] = magnetometerData.magnetic.z;

    currentMessage->BNOMessage.acceleration[0] = accelerometerData.acceleration.x;
    currentMessage->BNOMessage.acceleration[1] = accelerometerData.acceleration.y;
    currentMessage->BNOMessage.acceleration[2] = accelerometerData.acceleration.z;

    return true;
}

bool ReadLSM(Adafruit_LSM6DSO32 *LSM, GDQMessage_t *currentMessage, float accelBias[3], float gyroBias[3])
{
    //Sensor events
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    // if LSM read fails, return mutex, and schedule task again after 1 tick
    if(!LSM->getEvent(&accel, &gyro, &temp)) {
        return false;
    }

    // adds data to LSM struct
    currentMessage->LSMMessage.acceleration[0] = accel.acceleration.x - accelBias[0];
    currentMessage->LSMMessage.acceleration[1] = accel.acceleration.y - accelBias[1];
    currentMessage->LSMMessage.acceleration[2] = accel.acceleration.z - accelBias[2];

    currentMessage->LSMMessage.gyro[0] = gyro.gyro.x - gyroBias[0];
    currentMessage->LSMMessage.gyro[1] = gyro.gyro.y - gyroBias[1];
    currentMessage->LSMMessage.gyro[2] = gyro.gyro.z - gyroBias[2];

    return true;
}

bool ReadBMP(Adafruit_BMP5xx *BMP, GDQMessage_t *outputMsg, float* altBias)
{
    if(!BMP->performReading()) {
        return false;
    }

    // variable init
    float alt = 0;
    static float filteredAlt = 0;

    outputMsg->BMPMessage.temp = BMP->temperature;
    outputMsg->BMPMessage.pressure = BMP->pressure;

    // low pass filter altitude, update raw and filtered
    alt = BMP->readAltitude(1013.25f) - (*altBias);
    HWK_FILT_lowPass(alt, &filteredAlt, BMP_LOWPASS_ALPHA);
    outputMsg->BMPMessage.altitude = alt;
    outputMsg->BMPMessage.filteredAltitude = filteredAlt;

    return true;
}

bool updateSDBuffer(GDQMessage_t& currentMessage,
                    uint8_t* fsmState,
                    char msgBuf[],
                    size_t* index,
                    size_t bufSize)
{
    size_t remaining = bufSize - *index;

    int written = 0;

    if(fsmState != NULL)
    {
        written = snprintf(
            msgBuf + *index,
            remaining,
            "%i,%i\n",
            5,
            *fsmState
        );
    }
    else
    {
        switch(currentMessage.sensor)
        {
            case SENSOR_GPS:
                written = snprintf(
                    msgBuf + *index,
                    remaining,
                    "%i,%llu,%i,%f,%f,%c,%c,%f\n",
                    SENSOR_GPS,
                    currentMessage.GPSMessage.time,
                    currentMessage.GPSMessage.satellites,
                    currentMessage.GPSMessage.latitude,
                    currentMessage.GPSMessage.longitude,
                    currentMessage.GPSMessage.lat,
                    currentMessage.GPSMessage.lon,
                    currentMessage.GPSMessage.altitude
                );
                break;

            case SENSOR_ADXL:
                written = snprintf(
                    msgBuf + *index,
                    remaining,
                    "%i,%llu,%f,%f,%f\n",
                    SENSOR_ADXL,
                    currentMessage.ADXLMessage.time,
                    currentMessage.ADXLMessage.acceleration[0],
                    currentMessage.ADXLMessage.acceleration[1],
                    currentMessage.ADXLMessage.acceleration[2]
                );
                break;

            case SENSOR_BNO:
                written = snprintf(
                    msgBuf + *index,
                    remaining,
                    "%i,%llu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                    SENSOR_BNO,
                    currentMessage.BNOMessage.uptime,
                    currentMessage.BNOMessage.quaternion[0],
                    currentMessage.BNOMessage.quaternion[1],
                    currentMessage.BNOMessage.quaternion[2],
                    currentMessage.BNOMessage.quaternion[3],
                    currentMessage.BNOMessage.acceleration[0],
                    currentMessage.BNOMessage.acceleration[1],
                    currentMessage.BNOMessage.acceleration[2],
                    currentMessage.BNOMessage.euler_orientation[0],
                    currentMessage.BNOMessage.euler_orientation[1],
                    currentMessage.BNOMessage.euler_orientation[2],
                    currentMessage.BNOMessage.magnetometer[0],
                    currentMessage.BNOMessage.magnetometer[1],
                    currentMessage.BNOMessage.magnetometer[2]
                );
                break;

            case SENSOR_LSM:
                written = snprintf(
                    msgBuf + *index,
                    remaining,
                    "%i,%llu,%f,%f,%f,%f,%f,%f\n",
                    SENSOR_LSM,
                    currentMessage.LSMMessage.time,
                    currentMessage.LSMMessage.acceleration[0],
                    currentMessage.LSMMessage.acceleration[1],
                    currentMessage.LSMMessage.acceleration[2],
                    currentMessage.LSMMessage.gyro[0],
                    currentMessage.LSMMessage.gyro[1],
                    currentMessage.LSMMessage.gyro[2]
                );
                break;

            case SENSOR_BMP:
                written = snprintf(
                    msgBuf + *index,
                    remaining,
                    "%i,%llu,%f,%f,%f,%f\n",
                    SENSOR_BMP,
                    currentMessage.BMPMessage.time,
                    currentMessage.BMPMessage.temp,
                    currentMessage.BMPMessage.pressure,
                    currentMessage.BMPMessage.altitude,
                    currentMessage.BMPMessage.filteredAltitude
                );
                break;

            default:
                return true;
        }
    }

    // snprintf error
    if(written < 0)
    {
        return false;
    }

    // not enough room in buffer
    if((size_t)written >= remaining)
    {
        return false;
    }

    // advance write index
    *index += written;

    return true;
}

int Welford_Calibration(Welford_state *Welford, float calibration_data)
{
    if(Welford->n < 0) {
        return WELFORD_ERR_INVALID_SAMPLE_SIZE;
    }
    //set up for calculating the mean and variance
    Welford->n++;

    //determines how much a data value changes the Welford mean and variance
    float weighting_factor = calibration_data - Welford->mean;

    //update the mean before variance calculation
    Welford->mean += weighting_factor / (Welford->n);

    //makes a unit commonly refered to as "M2" or "S2" to easily compute variance
    Welford->mediary_unit += weighting_factor * (calibration_data - Welford->mean);
    
    //Calculate the variance    
    if ((Welford->n) != 1) {
        Welford->variance = (Welford->mediary_unit) / ((Welford->n) - 1);
        // this will live in the calibrate LSM function
        // if(Welford->variance >= LSM_GYRO_VARIANCE_THRESHOLD) {
        //     return WELFORD_ERR_VAR_DIVERGE;
        // }
    } else {
        Welford->variance = 0;
    }

    return WELFORD_SUCCESS;
}

bool calibrateIMUs(Adafruit_ADXL375* ADXL, Adafruit_LSM6DSO32* LSM, float ADXL_ACCEL_BIAS[3], float LSM_ACCEL_BIAS[3], float LSM_GYRO_BIAS[3], const int numSamples, const int divergenceThresh)
{
    GDQMessage_t currentReading;
    Welford_state adxlAccel[3];
    Welford_state lsmAccel[3];
    Welford_state lsmGyro[3];

    // archaic syntax, but initialize everything to zeroes
    memset(adxlAccel, 0, sizeof(adxlAccel));
    memset(lsmAccel, 0, sizeof(lsmAccel));
    memset(lsmGyro, 0, sizeof(lsmGyro));

    // start with ADXL
    currentReading.sensor = SENSOR_ADXL;

    // calibrate!
    for(int i = 0; i < numSamples; i++) {
        ReadADXL(ADXL, &currentReading, ADXL_ACCEL_BIAS);
        // TODO: add error handling
        Welford_Calibration(&adxlAccel[0], currentReading.ADXLMessage.acceleration[0]);
        Welford_Calibration(&adxlAccel[1], currentReading.ADXLMessage.acceleration[1]);
        Welford_Calibration(&adxlAccel[2], currentReading.ADXLMessage.acceleration[2]);

        // divergence check
        if (adxlAccel[0].variance > divergenceThresh
         || adxlAccel[1].variance > divergenceThresh
         || adxlAccel[2].variance > divergenceThresh)
        {
            return false;
        }
    }

    // now do LSM
    currentReading.sensor = SENSOR_LSM;

    for(int i = 0; i < numSamples; i++) {
        ReadLSM(LSM, &currentReading, LSM_ACCEL_BIAS, LSM_GYRO_BIAS);
        // TODO: add error handling
        Welford_Calibration(&lsmAccel[0], currentReading.LSMMessage.acceleration[0]);
        Welford_Calibration(&lsmAccel[1], currentReading.LSMMessage.acceleration[1]);
        Welford_Calibration(&lsmAccel[2], currentReading.LSMMessage.acceleration[2]);

        Welford_Calibration(&lsmGyro[0], currentReading.LSMMessage.gyro[0]);
        Welford_Calibration(&lsmGyro[1], currentReading.LSMMessage.gyro[1]);
        Welford_Calibration(&lsmGyro[2], currentReading.LSMMessage.gyro[2]);

        // divergence check
        if (lsmAccel[0].variance > divergenceThresh
         || lsmAccel[1].variance > divergenceThresh
         || lsmAccel[2].variance > divergenceThresh
         || lsmGyro[0].variance > divergenceThresh
         || lsmGyro[1].variance > divergenceThresh
         || lsmGyro[2].variance > divergenceThresh
        )
        {
            return false;
        }
    }

    // update biases
    ADXL_ACCEL_BIAS[0] = adxlAccel[0].mean;
    ADXL_ACCEL_BIAS[1] = adxlAccel[1].mean;
    ADXL_ACCEL_BIAS[2] = adxlAccel[2].mean;

    LSM_ACCEL_BIAS[0] = lsmAccel[0].mean;
    LSM_ACCEL_BIAS[1] = lsmAccel[1].mean;
    LSM_ACCEL_BIAS[2] = lsmAccel[2].mean;
    LSM_GYRO_BIAS[0] = lsmGyro[0].mean;
    LSM_GYRO_BIAS[1] = lsmGyro[1].mean;
    LSM_GYRO_BIAS[2] = lsmGyro[2].mean;

    return true;
}

bool calibrateAltimeter(Adafruit_BMP5xx *BMP, float *BMP_ALT_BIAS, const int numSamples, const int divergenceThresh)
{
    GDQMessage_t currentReading;
    Welford_state bmpAlt;

    // again archaic syntax but it works
    memset(&bmpAlt, 0, sizeof(bmpAlt));

    currentReading.sensor = SENSOR_BMP;
    
    // calibrate
    for(int i = 0; i < numSamples; i++) {
        ReadBMP(BMP, &currentReading, BMP_ALT_BIAS);
        // printf("BMP DATA: %f\n", currentReading.BMPMessage.altitude);
        Welford_Calibration(&bmpAlt, currentReading.BMPMessage.altitude);
    }
    
    //divergence check
    if(bmpAlt.variance > divergenceThresh) {
        return false;
    }
    
    // update bias
    *BMP_ALT_BIAS = bmpAlt.mean;

    return true;
}

int HWK_FILT_lowPass(float raw, float* filtered, const float alpha)
{
    // update lowpass IIR in place
    *filtered = alpha * raw + (1 - alpha) * (*filtered);

    // success
    return 0;
}


// returns true if it updates state
bool FSM(GDQMessage_t& currMsg, uint8_t& fsmState, float& apogeeEstimate)
{
    // init
    bool useLSM = false;
    static uint8_t eventCounter = 0;  // static to maintain, updated by helpers
    float accelMagADXL = 0;
    float accelMagLSM = 0;

    switch (fsmState) {
        case FSM_IDLE:
            // first, do we care about this message?
            if(!(currMsg.sensor == SENSOR_ADXL || currMsg.sensor == SENSOR_LSM))
            {
                // if not, skip
                break;
            }

            // next, is our current accelerometer the "active" one?
            if(currMsg.sensor == SENSOR_ADXL)
            {
                // is our current acceleration outside of the LSM's setpoint?
                accelMagADXL = sqrt(
                                 pow(currMsg.ADXLMessage.acceleration[0], 2) + 
                                 pow(currMsg.ADXLMessage.acceleration[1], 2) + 
                                 pow(currMsg.ADXLMessage.acceleration[2], 2)
                );

                // if LSM should be used,
                if (accelMagADXL <= 39.24)
                {
                    // exit early
                    break;
                }
                useLSM = false;
            } 
            else
            {
                accelMagLSM = sqrt(
                                pow(currMsg.LSMMessage.acceleration[0], 2) +
                                pow(currMsg.LSMMessage.acceleration[1], 2) +
                                pow(currMsg.LSMMessage.acceleration[2], 2)
                );

                // if LSM should be used,
                // TODO: what does data look like with LSM maxed?
                if (accelMagLSM >= 37.00)
                {
                    // exit early
                    break;
                }
                useLSM = true;
            }

            // if we're in LSM range,
            if (useLSM)
            {
                // feed in LSM data
                if(IdleToAscent(accelMagLSM, eventCounter))
                {
                    fsmState = FSM_ASCENT;
                    return true;
                }
            }

            // otherwise, use ADXL
            else
            {
                // feed in ADXL data
                if(IdleToAscent(accelMagADXL, eventCounter))
                {
                    fsmState = FSM_ASCENT;
                    return true;
                }
            }
            break;
                
        case FSM_ASCENT:
            // is this a sensor we care about?
            if (currMsg.sensor != SENSOR_BMP)
            {
                break;
            }
            
            // now run transition function
            if(AscentToDescent(currMsg.BMPMessage.filteredAltitude, currMsg.BMPMessage.time, eventCounter)) {
                fsmState = FSM_DESCENT;
                apogeeEstimate = currMsg.BMPMessage.filteredAltitude;
                return true;
            }

            break;
            
        case FSM_DESCENT:
            if(currMsg.sensor != SENSOR_BMP)
            {
                break;
            }
            if(DescentToLanded(currMsg.BMPMessage.filteredAltitude, eventCounter)) {
                fsmState = FSM_LANDED;
                return true;
            }
            break;
            

        // if we're landed, stop running FSM
        case FSM_LANDED:
            return true;
    }

    return false;
}

bool IdleToAscent(float currentAccelMag, uint8_t& counter)
{
    // if acceleration is above launch threshold, 
    if (currentAccelMag >= ASCENT_THRESHOLD)
    {
        // update counter
        counter++;
    } else
    {
        // reset counter if conditions aren't met
        counter = 0;
    }

    // if we've seen enough high accelerations read,
    if (counter == REQ_COUNT_STATE_CHANGE)
    {
        // reset counter   
        counter = 0;

        // change state
        return true;
    }

    // otherwise, state stays the same
    return false;
}

// more hawkeye previews
static void HWK_UTIL_updateRingBuffer(float buffer[], const uint8_t size, uint8_t* i, float newValue)
{
    // update value
    buffer[*i] = newValue;
    *i = (*i + 1) % size;   // modulus means index will wrap around when buffer is full

    return;
}

static void HWK_UTIL_unwrapRingBuffer(float inputBuffer[], float outputBuffer[], const uint8_t size, const uint8_t head)
{
    for (int i = 0; i < size; i++)
    {
        outputBuffer[i] = inputBuffer[(head + i) % size];
    }

    return;
}

// update, added it anyways, but use this later on
/**  not adding an unwrap function, use the following pattern:
 * for(uint8_t i = head; i < size; i++)
 * {
 *     process(ring[i]);
 * }
 *
 * for(uint8_t i = 0; i < head; i++)
 * {
 *     process(ring[i]);
 * }
**/

bool AscentToDescent(float altUpdate, uint64_t timeUpdate, uint8_t& counter)
{
    // init static altitude array
    static const uint8_t bufSize = REQ_COUNT_STATE_CHANGE;
    static float altitudes[bufSize];
    static float times[bufSize];
    static uint8_t altI;
    static uint8_t timeI;

    // add altitude/corresponding time to ring buffer
    HWK_UTIL_updateRingBuffer(altitudes, bufSize, &altI, altUpdate);
    HWK_UTIL_updateRingBuffer(times, bufSize, &timeI, (float)timeUpdate);

    uint8_t curr = (altI + bufSize - 1) % bufSize;
    uint8_t prev = (curr + bufSize - 1) % bufSize;

    // if altitude is increasing, reset counter
    if(altitudes[curr] > altitudes[prev])
    {
        counter = 0;
        return false;
    }

    // if altitude is decreasing, compute velocity
    // dt in microseconds, vel in seconds
    uint64_t dt = times[curr] - times[prev];
    float vel = ((altitudes[curr] - altitudes[prev]) / dt) / 1E6;

    // if velocity is negative enough, increase counter
    // if not, that must just be estimation noise since we've
    // already checked for a downward altitude trend
    if(vel < -0.5)
    {
        counter++;
    }

    // have we had enough successful samples?
    if(counter < REQ_COUNT_STATE_CHANGE)
    {
        return false;
    }

    // reset counter for next check
    counter = 0;
    return true;
}

// use BMP readings and Welford to see if we're on the ground
bool DescentToLanded(float altUpdate, uint8_t& counter)
{
    static Welford_state altState;

    // update running average
    Welford_Calibration(&altState, altUpdate);

    // check variance:
    if (altState.variance < LANDING_DIVERGENCE_THRESH)
    {
        counter++;
    }
    else
    {
        // reset state and counter
        counter = 0;
        memset(&altState, 0, sizeof(Welford_state));
        return false;
    }

    // have we ran long enough?
    if (counter < 4 * REQ_COUNT_STATE_CHANGE)
    {
        return false;
    }

    // reset counter and return true
    counter = 0;
    return true;
}