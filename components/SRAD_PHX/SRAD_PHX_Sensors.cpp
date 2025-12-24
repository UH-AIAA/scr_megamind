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
 * Reads the Adafruit LSM6DS032 6 DoF Accelerometer/Gyroscope.
 * It's index in the sensorStatus is 0.
 * @param LSM Referenece to initialized sensor instance
 * @returns Returns `true` if the operation succeeds, False if the operation fails
 */
uint8_t FLIGHT::read_LSM(Adafruit_LSM6DSO32 &LSM) {
    sensors_event_t accel, gyro, temp;

    // Attempt to read sensor data
    if(!LSM.getEvent(&accel, &gyro, &temp))
    {
        data.sensor_status[0] = 0;
        return 1;  // Return true if read fails
    }

    // Store gyroscope data
    data.lsm_gyro_x = gyro.gyro.x;
    data.lsm_gyro_y = gyro.gyro.y;
    data.lsm_gyro_z = gyro.gyro.z;

    // Store accelerometer data
    data.lsm_acc_x = accel.acceleration.x;
    data.lsm_acc_y = accel.acceleration.y;
    data.lsm_acc_z = accel.acceleration.z;

    // Store temperature data
    data.lsm_temp = float(temp.temperature);

    data.sensor_status[0] = 1;
    return 0;  // Return false if read succeeds
}

/**
 * Reads the Adafruit BMP388 Precision Barometer and Altimeter
 * It's index in sensorStatus is 1.
 * @param BMP Reference to initialized sensor instance\
 * @return Returns `true` if operation succeeds
 */
uint8_t FLIGHT::read_BMP(Adafruit_BMP3XX &BMP) {
    if (!BMP.performReading()) {
        data.sensor_status[1] = 0;
        return 1;
    }
    data.bmp_temp = BMP.temperature;
    data.bmp_press = BMP.pressure;

    if(STATE < STATES::FLIGHT_ASCENT) {
        data.bmp_alt = BMP.readAltitude(1013.25);   //uncalibrated/true altitude
    } else {
        data.bmp_alt = BMP.readAltitude(1013.25) - alt_offset;    //sea level can fluctuate under +/- 7 
                                                                    // depends on the data of the day. 
                                                                    //But 1013.25 is an acceptable value.
    }
    
    if(++altReadings_ind == 10) {
        altReadings_ind = 0;
    }
    altReadings[altReadings_ind] = data.bmp_alt;

    data.sensor_status[1] = 1;
    return 0;
}

/**
 * Reads the Adafruit ADXL_375 High-G Accelerometer
 * It's index in sensor status is 2.
 * @param ADXL Reference to initialized sensor instance
 * @return Returns `true`if operation succeeds
 */
uint8_t FLIGHT::read_ADXL(Adafruit_ADXL375 &ADXL) {
    sensors_event_t event;
    if (!ADXL.getEvent(&event)) {
        data.sensor_status[2] = 0;
        return 1;
    }
    data.adxl_acc_x = event.acceleration.x;
    data.adxl_acc_y = event.acceleration.y;
    data.adxl_acc_z = event.acceleration.z;

    data.adxl_temp = float(event.temperature);

    data.sensor_status[2] = 1;
    return 0;
}

/**
 * Returns Adafruit BNO055 Absolute Orientation Sensor
 * It's index in sensorStatus is 3.
 * @param BNO Initialized sensor instance
 * @return Returns `true` if operation succeeds
 */
uint8_t FLIGHT::read_BNO(Adafruit_BNO055 &BNO) {
    sensors_event_t orientationData, angVelocityData, magnetometerData, accelerometerData;

    if (!BNO.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER)) {
        data.sensor_status[3] = 0;
        return 1;
    }
    if (!BNO.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
        data.sensor_status[3] = 0;
        return 1;
    }
    if (!BNO.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER)) {
        data.sensor_status[3] = 0;
        return 1;
    }
    if (!BNO.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER)) {
        data.sensor_status[3] = 0;
        return 1;
    }

    imu::Quaternion quat = BNO.getQuat();
    data.bno_ori_w = quat.w();
    data.bno_ori_x = quat.x();
    data.bno_ori_y= quat.y();
    data.bno_ori_z = quat.z();

    data.bno_gyro_x = angVelocityData.gyro.x;
    data.bno_gyro_y = angVelocityData.gyro.y;
    data.bno_gyro_z = angVelocityData.gyro.z;

    data.bno_acc_x = accelerometerData.acceleration.x;
    data.bno_acc_y = accelerometerData.acceleration.y;
    data.bno_acc_z = accelerometerData.acceleration.z;

    data.bno_temp = float(BNO.getTemp());

    data.sensor_status[3] = 1;
    return 0;
}

/**
 * Reads Adafruit Ultimate GPS Breakout V3
 * It's index in sensorStatus is 4.
 * @param GPS Initialized Sensor instance
 * @return Returns `false` if GPS isn't ready in 500ms or no satellite fix, returns `true` otherwise
 */
uint8_t FLIGHT::read_GPS(Adafruit_GPS &GPS) {
    last_gps = &GPS;
    
    uint32_t startms = millis();
    uint32_t timeout = startms + 150;

    while (millis() < timeout) {
        while (GPS.available()) {
            GPS.read();

            if (GPS.newNMEAreceived()) {
                // Serial.println(GPS.lastNMEA());
                if (!GPS.parse(GPS.lastNMEA())) {
                    continue;
                }

                if (GPS.fix && GPS.satellites > 0) {
                    // Serial.print("Satellites: ");
                    // Serial.println(GPS.satellites);
                    data.sensor_status[4] = 0;
                    return 0;
                }
            }
        }
    }

    data.sensor_status[4] = 1;
    return 1;
}

