// Nathan Samuell C 2026
#include "megamind.h"

// returns true on success, false on failure, TODO: [NS] add error modes to this refactor
bool ReadADXL(Adafruit_ADXL375* ADXL, GDQMessage_t *outputMsg)
{
    static sensors_event_t event;
    // if read operation fails, start task over:
    if(!ADXL->getEvent(&event)) {
        // xSemaphoreGive(sensor_spi_mutex);
        return false;
    }

    // save off our sensor data, add it to queue
    outputMsg->ADXLMessage.acceleration[0] = event.acceleration.x;
    outputMsg->ADXLMessage.acceleration[1] = event.acceleration.y;
    outputMsg->ADXLMessage.acceleration[2] = event.acceleration.z;

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

bool ReadLSM(Adafruit_LSM6DSO32 *LSM, GDQMessage_t *currentMessage)
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
    currentMessage->LSMMessage.acceleration[0] = accel.acceleration.x;
    currentMessage->LSMMessage.acceleration[1] = accel.acceleration.y;
    currentMessage->LSMMessage.acceleration[2] = accel.acceleration.z;

    currentMessage->LSMMessage.gyro[0] = gyro.gyro.x;
    currentMessage->LSMMessage.gyro[1] = gyro.gyro.y;
    currentMessage->LSMMessage.gyro[2] = gyro.gyro.z;

    return true;
}

bool ReadBMP(Adafruit_BMP5xx *BMP, GDQMessage_t *outputMsg)
{
    if(!BMP->performReading()) {
        return false;
    }

    outputMsg->BMPMessage.temp = BMP->temperature;
    outputMsg->BMPMessage.pressure = BMP->pressure;
    outputMsg->BMPMessage.altitude = BMP->readAltitude(1013.25f);

    return true;
}