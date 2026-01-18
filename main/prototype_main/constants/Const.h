#include <stdint.h>

/// Define frequency Lora + SD
#define LORA_FREQ 915E6 /// 915MHz frequency for Lora
#define SD_RATE 20E6    /// 20MHz rate for SD 

/// @brief  Sensors helper constants
/*
    MAX_GPS_BYTES_PER_LOOP:             Capped amount of bytes permitted when reading GPS data (tune as needed)
    GPS_TIMEOUT:                        Capped amount of timeout (ms) for GPS to read data
    ADXL_SAMPLES_MAX:                   Maximum data samples for calibration
    LSM_SAMPLES_MAX:                    Maximum data samples for calibration
    BMP_SAMPLES_MAX:                    Maximum data samples for calibration
*/
const uint8_t MAX_GPS_BYTES_PER_LOOP = 64;  
const uint8_t GPS_TIMEOUT = 200;
const uint8_t ADXL_SAMPLES_MAX = 20; 
const uint8_t LSM_SAMPLES_MAX = 20;
const uint8_t BMP_SAMPLES_MAX = 20; 

/// @brief: Constant for state machine
/*
    ACCEL_LAUNCH_G:                     G force multiplier for state change detection
    GRAVITY_FORCE:                      Constant of gravity force on the Earth
    JUNO_MAX_SPEED:                     Current max speed of Juno in m/s
    REQ_COUNT_STATE_CHANGE:             Amount of check before state is allowed to change
    BMP_DATA_RATE:                      Current data rate of BMP 581 in seconds
    BMP_STANDARD_DEVIATION:             Sample standard deviation of 50 samples of BMP data
    BMP_NOISE_MULTIPLIER:               Multiplier factor to guard actual physical change from noise (99.73%)
    BMP_DESCEND_THRESHOLD:              Threshold describe the require distance between apogee to 
                                            current altitude when rocket is going down to trigger state change
    ASCEND_THRESHOLD:                   Threshold to allow ascend detection at 2g = 19.6 m/s^2
    BMP_STEP_MAX:                       Maximum step in data of the BMP that is physically possible with its current tested rate of 28.57 [Hz]
    BMP_NOISE_THRESHOLD:                Threshold to guard change in altitude from noise 
    BMP_LAND_THRESHOLD:                 Threshold to determine if the rocket is landed based of the altitude. 
                                            Average of launch site at Space Port is: 885 m
                                            Highest altitude around 5 mile radius from launch site is: 922.6 m
                                            Juno height is: 2.97 m
                                            Land threshold = ceil(Alt_max - alt_avg_start + Juno's height)
    ADXL_MAGNITUDE_STANDARD_DEVIATION:  Sample standard deviation of ADXL acceleration magnitude from 51 samples
    LSM_MAGNITUDE_STANDARD_DEVIATION:   Sample standard deviation of LSM acceleration magnitude from 50 samples
    LAND_NOISE_MULTIPLIER:              Multiplier factor to maximize noise + wind dragging the rocket when it is landed
    ADXL_LAND_THRESHOLD:                Land threshold for ADXL acceleration magnitude. If magnitude < magnitude threshold -> ~landed
    LSM_LAND_THRESHOLD:                 Land threshold for LSM acceleration magnitude. If magnitude < magnitude threshold -> ~landed
    LAND_COUNTER_MAX:                   Fail safe constant if all sensors for state switching of landing fail. 
                                        300 iteration is ~62.4 secs at rate of 4.81 [Hz] for state machine task to switch to land if all sensors fail

    General formula used:               v = d/t, d = vt (Newton is proud of us)
                                        Noise threshold = Multiplier factor * standard deviation
*/
// TODO: [NS] think about making these #define 
const uint8_t ACCEL_LAUNCH_G = 2; 
const float GRAVITY_FORCE = 9.80665;
const float JUNO_MAX_SPEED = 301.483;
const uint8_t REQ_COUNT_STATE_CHANGE = 3;
const float BMP_DATA_RATE = 0.0355;
const float BMP_STANDARD_DEVIATION = 0.07594;
const uint8_t BMP_NOISE_MULTIPLIER = 3;
const uint8_t BMP_DESCEND_THRESHOLD = 20;
const float ASCEND_THRESHOLD = GRAVITY_FORCE * ACCEL_LAUNCH_G;
const float BMP_STEP_MAX = JUNO_MAX_SPEED * BMP_DATA_RATE;
const float BMP_NOISE_THRESHOLD = BMP_STANDARD_DEVIATION * BMP_NOISE_MULTIPLIER;
const uint8_t BMP_LAND_THRESHOLD = 41;
const float ADXL_MAGNITUDE_STANDARD_DEVIATION = 0.698823;
const float LSM_MAGNITUDE_STANDARD_DEVIATION = 0.0005284;
const float LAND_NOISE_MULTIPLIER = 3.5;
const float ADXL_LAND_THRESHOLD = ADXL_MAGNITUDE_STANDARD_DEVIATION * LAND_NOISE_MULTIPLIER;
const float LSM_LAND_THRESHOLD = LSM_MAGNITUDE_STANDARD_DEVIATION * LAND_NOISE_MULTIPLIER;
const uint16_t LAND_COUNTER_MAX = 300;