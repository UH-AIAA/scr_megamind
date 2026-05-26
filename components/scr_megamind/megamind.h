// Nathan Samuell Copyright 2026

#include <Arduino.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_LSM6DSO32.h>


////////////////////////////////
/*      TYPE DEFINITIONS      */
////////////////////////////////
typedef struct ADXLMessage {
    uint64_t time;          // ms since start for now (what we're already doing), maybe move to unixtime/RTC later
    float acceleration[3];  // acceleration data (x, y, z);
} ADXLMessage_t;

typedef struct BNOMessage {
    uint64_t uptime;              //uptime from BNO snsor
    float quaternion[4];          //W,X,Y,Z data
    float euler_orientation[3];   //x,y,z angular acceleration
    float magnetometer[3];        //x,y,z, of SOMETHING, not sure yet
    float acceleration[3];        //x,y,z linear acceleration
} BNOMessage_t;

typedef struct GPSMessage {
    uint64_t time;
    int satellites;               // not sure if this is necessary but I'll include it
    float longitude, latitude;    // numerical coordinates for the longitude and latitude in degress/minutes
    char lon,lat;                 // stores the cardinal direction (E/W for lon and N/S for lat)
    float altitude;               // gps altitude reading
}GPSMessage_t;

typedef struct LSMMessage {
    uint64_t time;
    float acceleration[3];         // stores the X,Y,Z acceleration
    float gyro[3];                 // stores X,Y,Z gyro orientation
    //float temp;                  // this was commented off but I'll keep it here in case we use it
}LSMMessage_t;

typedef struct BMPMessage {
    uint64_t time;
    float temp, pressure, altitude;    // temperature in celcius, pressure in pascals, altitude from sea level
}BMPMessage_t;

#pragma pack(push, 1)
typedef struct LORAMessage {
    uint64_t BMP_time;
    int16_t BMP_temp, BMP_pressure, BMP_altitude;

    uint64_t LSM_time;
    int16_t LSM_accel[3];
    int16_t LSM_gyro[3];

    uint64_t ADXL_time;
    int16_t ADXL_accel[3];

    uint64_t BNO_time;
    int16_t BNO_quat[4];
    int16_t BNO_euler[3];
    int16_t BNO_magnet[3];
    int16_t BNO_accel[3];

    uint64_t GPS_time;
    int8_t GPS_sat;
    int16_t GPS_lon, GPS_lat;
    char GPS_lon_dir, GPS_lat_dir;
    int16_t GPS_alt;

    // empty for now, here for thanh's ground test. will populate pending state machine integration
    uint8_t flightState;
    float apogeeEstimate;
} LORAMessage_t;
#pragma pack(pop)

typedef enum SensorType {
    SENSOR_GPS = 0,
    SENSOR_ADXL = 1,
    SENSOR_BNO  = 2,
    SENSOR_LSM  = 3,
    SENSOR_BMP  = 4,
} SensorType_t;

typedef struct GDQMessage {
    // tentatively removed, using the individual message time bugs as well
    // uint32_t time;
    SensorType_t sensor;

    union {
        GPSMessage_t GPSMessage;
        ADXLMessage_t ADXLMessage;
        BNOMessage_t BNOMessage;
        LSMMessage_t LSMMessage;
        BMPMessage_t BMPMessage;
    };
} GDQMessage_t;

// SENSOR DATA STORAGE QUEUES
// GDQ == Global Data Queue
typedef struct GDQ {
    QueueHandle_t SensorQueue;

    GPSMessage_t LatestGPSMsg;    // added to make globally available
    ADXLMessage_t LatestADXLMsg;
    BNOMessage_t LatestBNOMsg;
    LSMMessage_t LatestLSMMsg;
    BMPMessage_t LatestBMPMsg;
} GDQ_t;

// calibration data types
//Defining a struct to keep relevant data together update them
typedef struct {
    uint32_t n;
    float mean;
    float variance;
    float mediary_unit;
} Welford_state;

//Enumerating variables as constants equivalent to error codes
typedef enum {
    WELFORD_SUCCESS                 = 0,    // success
    WELFORD_ERR_DBZ                 = 1,    // divide by zero error
    WELFORD_ERR_INVALID_SAMPLE_SIZE = 2,    // too few samples (<0)
    WELFORD_ERR_VAR_DIVERGE         = 3,    // variance > threshold
};

////////////////////////////////
/*    FUNCTION DEFINITIONS    */
////////////////////////////////

// sensor functions
bool ReadADXL(Adafruit_ADXL375* ADXL, GDQMessage_t *outputMsg, float accelBias[3]);
bool ReadBNO(Adafruit_BNO055 *BNO, GDQMessage_t *currentMessage);
bool ReadLSM(Adafruit_LSM6DSO32 *LSM, GDQMessage_t *currentMessage, float accelBias[3], float gyroBias[3]);
bool ReadBMP(Adafruit_BMP5xx *BMP, GDQMessage_t *outputMsg, float* altBias);

// calibration helpers
int Welford_Calibration(Welford_state *Welford, float calibration_data);
bool calibrateIMUs(Adafruit_ADXL375* ADXL, Adafruit_LSM6DSO32* LSM, float ADXL_ACCEL_BIAS[3], float LSM_ACCEL_BIAS[3], float LSM_GYRO_BIAS[3], const int numSamples, const int divergenceThresh);
bool calibrateAltimeter(Adafruit_BMP5xx *BMP, float *BMP_BIAS, const int numSamples, const int divergenceThresh);