#ifndef SRAD_PHX_H
#define SRAD_PHX_H

#include <Arduino.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>

struct TelemetryData { // Easy transfer can only work with basic data types 
                      //(int, float, etc.. but not vector3 stuff due to unpredictability)
    float lsm_gyro_x, lsm_gyro_y, lsm_gyro_z;
    float lsm_acc_x, lsm_acc_y, lsm_acc_z;
    float adxl_acc_x, adxl_acc_y, adxl_acc_z;
    float bno_gyro_x, bno_gyro_y, bno_gyro_z;
    float bno_acc_x, bno_acc_y, bno_acc_z;
    float bno_mag_x, bno_mag_y, bno_mag_z;
    float bno_ori_w, bno_ori_x, bno_ori_y, bno_ori_z;
    float lsm_temp, adxl_temp, bno_temp, bmp_temp;
    float bmp_press, bmp_alt;

    uint8_t sensor_status[5];
};

enum STATES {
    PRE_NO_CAL = 0,
    PRE_CAL = 1,
    FLIGHT_ASCENT = 2,
    FLIGHT_DESCENT = 3,
    POST_LANDED = 4,
};

class FLIGHT {
    public:
        // three stack initial constructor
        FLIGHT(int a1, int a2, int l1, int l2, String h, Adafruit_GPS& g, TelemetryData& o) 
        : accel_liftoff_threshold(a1), accel_liftoff_time_threshold(a2), 
        land_time_threshold(l1), land_altitude_threshold(l2), data_header(h), last_gps(&g), data(o) {
            STATE = STATES::PRE_NO_CAL;
            runningTime_ms = 0;

            // initialize arrays!
            altReadings_ind = 0;
            for(int i = 0; i < 10; i++) {
                altReadings[i] = 0;
            }
        }

        // UART Constructor
        FLIGHT(String h, Adafruit_GPS& g, TelemetryData& o) 
        : data_header(h), last_gps(&g), data(o) {
            STATE = STATES::PRE_NO_CAL;
            runningTime_ms = 0;
        }

        // SPI Constructor
        FLIGHT(int a1, int a2, int l1, int l2, String h, TelemetryData& o) 
        : accel_liftoff_threshold(a1), accel_liftoff_time_threshold(a2), 
        land_time_threshold(l1), land_altitude_threshold(l2), data_header(h), data(o) {
            STATE = STATES::PRE_NO_CAL;
            runningTime_ms = 0;
            last_gps = nullptr;

            // initialize arrays!
            altReadings_ind = 0;
            for(int i = 0; i < 10; i++) {
                altReadings[i] = 0;
            }
        }

        // high level functions
        void calculateState();
        uint8_t read_LSM(Adafruit_LSM6DSO32 &);
        uint8_t read_BMP(Adafruit_BMP3XX &);
        uint8_t read_ADXL(Adafruit_ADXL375 &);
        uint8_t read_BNO(Adafruit_BNO055 &);
        uint8_t read_GPS(Adafruit_GPS &);
        void incrementTime();
        void writeSD(bool, File &);
        void writeSERIAL(bool, Stream &);  // Stream allows Teensy USB as well
        void writeDataToTeensy(); //no stream parameter needed for EasyTransfer
        void readDataFromTeensy(); //no stream parameter needed for EasyTransfer
        void writeDEBUG(bool, Stream &);


        // helper functions
        bool isCal();
        bool isAscent();
        bool isDescent();
        bool isLanded();
        bool calibrate();

        void initTransferSerial(Stream &);
        void AltitudeCalibrate();
        void printRate();

    private:
        int accel_liftoff_threshold;        // METERS PER SECOND^2
        int accel_liftoff_time_threshold;   // MILLISECONDS
        int land_time_threshold;            // MILLISECONDS
        int land_altitude_threshold;        // METERS

        String data_header;
        Adafruit_GPS* last_gps;             // used for data collection, for some reason the GPS stores it
        uint16_t deltaTime_ms;
        uint64_t runningTime_ms;

        // data processing variables
        float alt_offset;                   // DO NOT MODIFY
        float prev_alt, v_vel, offset_alt_fixed_temp;
        bool offset_calibrated;             // flag to tell us if we've configured this

        float altReadings[10];
        uint8_t altReadings_ind;


        bool calibrated = false;
        STATES STATE;

        // EasyTransfer ET;
        TelemetryData* txData;
        TelemetryData* rxData;
        TelemetryData data;
};

#endif
