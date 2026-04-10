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
    outputMsg->ADXLMessage.acceleration[0] = event.acceleration.x - accelBias[0];
    outputMsg->ADXLMessage.acceleration[1] = event.acceleration.y - accelBias[1];
    outputMsg->ADXLMessage.acceleration[2] = event.acceleration.z - accelBias[2];

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

    outputMsg->BMPMessage.temp = BMP->temperature;
    outputMsg->BMPMessage.pressure = BMP->pressure;
    outputMsg->BMPMessage.altitude = BMP->readAltitude(1013.25f) - (*altBias);

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
        Welford->variance = (Welford->mediary_unit) / (Welford->n);
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
    ADXL_ACCEL_BIAS[1] = adxlAccel[2].mean;

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

    currentReading.sensor = SENSOR_BMP;
    
    // calibrate
    for(int i = 0; i < numSamples; i++) {
        ReadBMP(BMP, &currentReading, BMP_ALT_BIAS);
        Welford_Calibration(&bmpAlt, currentReading.BMPMessage.altitude);
    }
    
    //divergence check
    if(bmpAlt.mean > divergenceThresh) {
        return false;
    }
    
    // update bias
    *BMP_ALT_BIAS = bmpAlt.mean;

    return true;
}