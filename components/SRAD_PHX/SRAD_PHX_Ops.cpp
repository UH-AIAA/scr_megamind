/* SRAD Avionics Flight Software for AIAA-UH
 *
 * Copyright (c) 2025 Nathan Samuell + Dedah + Thanh! (www.github.com/nathansamuell, www.github.com/UH-AIAA)
 *
 * More information on the MIT license as well as a complete copy
 * of the license can be found here: https://choosealicense.com/licenses/mit/
 *
 * All above text must be included in any redistribution.
 */

#include "SRAD_PHX.h"

/** 
 * @brief tracks time during flight
 * 
 * This function calls the millis() function from the
 * Arduino library to track milliseconds since the board
 * was powered on. It updates three variables: 
 * 
 * 1. `deltaTime_ms`
 * 2. `runningTime_ms`
 * 3. `output.totalTime_ms`
 */
void FLIGHT::incrementTime() {
    uint64_t newRunningTime_ms = millis();
    deltaTime_ms = newRunningTime_ms - runningTime_ms;
    runningTime_ms = newRunningTime_ms;
}

/**
 * @brief writes data stored in `output` to file
 * @param headers If true, function will only right headers and return early
 * @param File A reference to Arduino file type from SD.h
 * 
 * This function can write data headers or current data to SD card.
 */
void FLIGHT::writeSD(bool headers, File& outputFile) {
    if(headers) {
        outputFile.println(data_header);
        outputFile.flush();
        return;
    }

    outputFile.print(runningTime_ms); outputFile.print(", ");
    if(last_gps != nullptr) {
        if(last_gps->fix) {
            outputFile.print(last_gps->latitudeDegrees, 6); outputFile.print(", ");
            outputFile.print(last_gps->longitudeDegrees, 6); outputFile.print(",");
            outputFile.print((int32_t)last_gps->satellites); outputFile.print(",");
            outputFile.print(last_gps->speed, 3); outputFile.print(",");
            outputFile.print(last_gps->angle, 3); outputFile.print(",");
            outputFile.print(last_gps->altitude, 3); outputFile.print(",");
        } else {
            outputFile.print("-1,No fix,-1,No fix,0,-1,-1,-1,");
        }
    }
    outputFile.print(data.bno_ori_w, 5); outputFile.print(",");
    outputFile.print(data.bno_ori_x, 5); outputFile.print(",");
    outputFile.print(data.bno_ori_y, 5); outputFile.print(",");
    outputFile.print(data.bno_ori_z, 5); outputFile.print(",");
    outputFile.print(data.bno_gyro_x, 5); outputFile.print(",");
    outputFile.print(data.bno_gyro_y, 5); outputFile.print(",");
    outputFile.print(data.bno_gyro_z, 5); outputFile.print(",");
    outputFile.print(data.bno_acc_x, 4); outputFile.print(",");
    outputFile.print(data.bno_acc_y, 4); outputFile.print(",");
    outputFile.print(data.bno_acc_z, 4); outputFile.print(",");
    outputFile.print(data.adxl_acc_x, 2); outputFile.print(",");
    outputFile.print(data.adxl_acc_y, 2); outputFile.print(",");
    outputFile.print(data.adxl_acc_z, 2); outputFile.print(",");
    outputFile.print(data.bmp_press, 6); outputFile.print(",");
    outputFile.print(data.bmp_alt, 4); outputFile.print(",");
    outputFile.print(data.lsm_temp, 2); outputFile.print(",");
    outputFile.print(data.adxl_temp, 2); outputFile.print(",");
    outputFile.print(data.bno_temp, 2); outputFile.print(",");
    outputFile.print(data.bmp_temp, 2); outputFile.print(",");
    outputFile.print(data.sensor_status[0]); outputFile.print(",");
    outputFile.print(data.sensor_status[1]); outputFile.print(",");
    outputFile.print(data.sensor_status[2]); outputFile.print(",");
    outputFile.print(data.sensor_status[3]); outputFile.print(",");
    outputFile.print(data.sensor_status[4]); outputFile.println();
    outputFile.flush();

    return;
}

/**
 * @brief writes data stored in `output` to a serial port
 * @param headers If true, function will only right headers and return early
 * @param Serial1 The serial port to write data to
 * 
 * This function can write data headers or current data to a serial port.
 */
void FLIGHT::writeSERIAL(bool headers, Stream& outputSerial) {
    if(headers) {
        outputSerial.println(data_header);
        outputSerial.flush();
        return;
    }

    outputSerial.print(runningTime_ms); outputSerial.print(",");
    if(last_gps != nullptr) {
        if(last_gps->fix) {
            outputSerial.print(last_gps->latitudeDegrees, 6); outputSerial.print(",");
            outputSerial.print(last_gps->longitudeDegrees, 6); outputSerial.print(",");
            outputSerial.print((int32_t)last_gps->satellites); outputSerial.print(",");
            outputSerial.print(last_gps->speed, 3); outputSerial.print(",");
            outputSerial.print(last_gps->angle, 3); outputSerial.print(",");
            outputSerial.print(last_gps->altitude, 3); outputSerial.print(",");
        } else {
            outputSerial.print("-1,No fix,-1,No fix,0,-1,-1,-1,");
        }
    }
    outputSerial.print(data.lsm_gyro_x, 5); outputSerial.print(",");
    outputSerial.print(data.lsm_gyro_y, 5); outputSerial.print(",");
    outputSerial.print(data.lsm_gyro_z, 5); outputSerial.print(",");
    outputSerial.print(data.bno_ori_w, 5); outputSerial.print(",");
    outputSerial.print(data.bno_ori_x, 5); outputSerial.print(",");
    outputSerial.print(data.bno_ori_y, 5); outputSerial.print(",");
    outputSerial.print(data.bno_ori_z, 5); outputSerial.print(",");
    outputSerial.print(data.bno_gyro_x, 5); outputSerial.print(",");
    outputSerial.print(data.bno_gyro_y, 5); outputSerial.print(",");
    outputSerial.print(data.bno_gyro_z, 5); outputSerial.print(",");
    outputSerial.print(data.bno_acc_x, 4); outputSerial.print(",");
    outputSerial.print(data.bno_acc_y, 4); outputSerial.print(",");
    outputSerial.print(data.bno_acc_z, 4); outputSerial.print(",");
    outputSerial.print(data.adxl_acc_x, 2); outputSerial.print(",");
    outputSerial.print(data.adxl_acc_y, 2); outputSerial.print(",");
    outputSerial.print(data.adxl_acc_z, 2); outputSerial.print(",");
    outputSerial.print(data.bmp_press, 6); outputSerial.print(",");
    outputSerial.print(data.bmp_alt, 4); outputSerial.print(",");
    outputSerial.print(data.lsm_temp, 2); outputSerial.print(",");
    outputSerial.print(data.adxl_temp, 2); outputSerial.print(",");
    outputSerial.print(data.bno_temp, 2); outputSerial.print(",");
    outputSerial.print(data.bmp_temp, 2); outputSerial.print(",");
    outputSerial.print(data.sensor_status[0]); outputSerial.print(",");
    outputSerial.print(data.sensor_status[1]); outputSerial.print(",");
    outputSerial.print(data.sensor_status[2]); outputSerial.print(",");
    outputSerial.print(data.sensor_status[3]); outputSerial.print(",");
    outputSerial.print(data.sensor_status[4]); outputSerial.println();
    outputSerial.flush();

    return;
}

void FLIGHT::writeDEBUG(bool headers, Stream &outputSerial) {
    if(headers) {
        outputSerial.println(data_header);
        outputSerial.flush();
        return;
    }

    outputSerial.print("Uptime (ms): ");outputSerial.print(runningTime_ms); outputSerial.print(", \n");
    outputSerial.print("State: "); outputSerial.println(STATE); outputSerial.println("\n");
    if(last_gps != nullptr) {
        if(last_gps->fix) {
            outputSerial.print("GPS Latitude Degrees: ");outputSerial.print(last_gps->latitudeDegrees, 6); outputSerial.println(", ");
            outputSerial.print("GPS Longitude Degrees: ");outputSerial.print(last_gps->longitudeDegrees, 6); outputSerial.println(",");
            outputSerial.print("GPS satellites: ");outputSerial.print((int32_t)last_gps->satellites); outputSerial.print(",");
            outputSerial.print("GPS speed: ");outputSerial.print(last_gps->speed, 3); outputSerial.print(",");
            outputSerial.print("GPS angle: ");outputSerial.print(last_gps->angle, 3); outputSerial.print(",");
            outputSerial.print("GPS altitude: ");outputSerial.println(last_gps->altitude, 3); outputSerial.println();
        } else {
            outputSerial.println("-1,No fix,-1,No fix,0,-1,-1,-1,\n");
        }
    }
    // LSM data
    outputSerial.print("LSM Gyro X: "); outputSerial.print(data.lsm_gyro_x, 5); outputSerial.print(",");
    outputSerial.print("LSM Gyro Y: "); outputSerial.print(data.lsm_gyro_y, 5); outputSerial.print(",");
    outputSerial.print("LSM Gyro Z: "); outputSerial.print(data.lsm_gyro_z, 5); outputSerial.println(",");

    outputSerial.print("LSM Acc X: "); outputSerial.print(data.lsm_acc_x, 5); outputSerial.print(",");
    outputSerial.print("LSM Acc Y: "); outputSerial.print(data.lsm_acc_y, 5); outputSerial.print(",");
    outputSerial.print("LSM Acc Z: "); outputSerial.print(data.lsm_acc_z, 5); outputSerial.println(",");

    //BNO data
        //orientation
    outputSerial.print("BNO W-Orientation: ");outputSerial.print(data.bno_ori_w, 5); outputSerial.print(",");
    outputSerial.print("BNO X-Orientation: ");outputSerial.print(data.bno_ori_x, 5); outputSerial.print(",");
    outputSerial.print("BNO Y-Orientation: ");outputSerial.print(data.bno_ori_y, 5); outputSerial.print(",");
    outputSerial.print("BNO Z-Orientation: ");outputSerial.print(data.bno_ori_z, 5); outputSerial.println(",");
        //gyro
    outputSerial.print("BNO X-Gyro: ");outputSerial.print(data.bno_gyro_x, 5); outputSerial.print(",");
    outputSerial.print("BNO Y-Gyro: ");outputSerial.print(data.bno_gyro_y, 5); outputSerial.print(",");
    outputSerial.print("BNO Z-Gyro: ");outputSerial.print(data.bno_gyro_z, 5); outputSerial.println(",");
        //Accel
    outputSerial.print("BNO X-Accel: ");outputSerial.print(data.bno_acc_x, 4); outputSerial.print(",");
    outputSerial.print("BNO Y-Accel: ");outputSerial.print(data.bno_acc_y, 4); outputSerial.print(",");
    outputSerial.print("BNO Z-Accel: ");outputSerial.print(data.bno_acc_z, 4); outputSerial.println(",");

    //ADXL data
    outputSerial.print("ADXL X_Accel: ");outputSerial.print(data.adxl_acc_x, 2); outputSerial.print(",");
    outputSerial.print("ADXL Y_Accel: ");outputSerial.print(data.adxl_acc_y, 2); outputSerial.print(",");
    outputSerial.print("ADXL Z_Accel: ");outputSerial.print(data.adxl_acc_z, 2); outputSerial.println(",");

    //BMP data
    outputSerial.print("BMP Pressure: ");outputSerial.print(data.bmp_press, 6); outputSerial.print(",");
    outputSerial.print("BMP Altitude: ");outputSerial.print(data.bmp_alt, 4); outputSerial.println(",");

    //Temperature data
    outputSerial.print("LSM Temp: ");outputSerial.print(data.lsm_temp, 2); outputSerial.print(",");
    outputSerial.print("ADXL Temp: ");outputSerial.print(data.adxl_temp, 2); outputSerial.print(",");
    outputSerial.print("BNO Temp: ");outputSerial.print(data.bno_temp, 2); outputSerial.print(",");
    outputSerial.print("BMP Temp: ");outputSerial.print(data.bmp_temp, 2); outputSerial.println("\n");

    //Sensor status
    outputSerial.println("Sensor Status:");
    outputSerial.print(data.sensor_status[0]); outputSerial.print(", ");
    outputSerial.print(data.sensor_status[1]); outputSerial.print(", ");
    outputSerial.print(data.sensor_status[2]); outputSerial.print(", ");
    outputSerial.print(data.sensor_status[3]); outputSerial.print(", ");
    outputSerial.print(data.sensor_status[4]); outputSerial.println("\n");
    outputSerial.flush();

    return;
}

/* This block is old teensy-to-teensy functions
//////////////////////////////////////////////////////////////////////////////
void FLIGHT::writeDataToTeensy() {
    Serial.println("Transmitting");
    ET.sendData();
}

void FLIGHT::readDataFromTeensy() {
    uint64_t startTime = millis();
    while(millis() < startTime + 1000) {
        if (ET.receiveData()) {
            #ifdef DEBUG
                Serial.println("Data Received!");
            #endif
            return;
        } else {
            #ifdef DEBUG
                Serial.print("No Data Received, time since waiting: ");
                Serial.println(millis() - startTime);
                delay(25);
            #endif
        }
    }
}

void FLIGHT::initTransferSerial(Stream &transferSerial) {
    ET.begin(details(data), &transferSerial);
}
//////////////////////////////////////////////////////////////////////////////
*/

void FLIGHT::printRate() {
    Serial.print("Cycle Time: "); Serial.println(deltaTime_ms);
    Serial.print("Cycle Rate: "); Serial.println(1000.0/float(deltaTime_ms));
}
