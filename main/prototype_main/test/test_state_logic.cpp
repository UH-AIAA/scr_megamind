#include <gtest/gtest.h>
#include "../include/prototype_main.h"
#include "../constants/Const.h"

/// ----------------------IDLE to ASCEND test------------------///
TEST(StateMachineTest, AdxlIdleToDescendOk) {
    OutputData_t OutputData = {0};
    MagnitudeData_t MagnitudeData = {0};
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    OutputData.bmp_alt = 10.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    OutputData.sensorStatus.adxl_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
    EXPECT_EQ(peakAltitude, 12);
}

TEST(StateMachineTest, AdxlIdleToAscendMagnitudeDropFail){
    OutputData_t OutputData = {0};
    MagnitudeData_t MagnitudeData = {0};
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    OutputData.sensorStatus.adxl_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    MagnitudeData.adxl_accel_magnitude = 10;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, LsmIdleToDescendOk) {
    OutputData_t OutputData = {0};
    MagnitudeData_t MagnitudeData = {0};
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    OutputData.bmp_alt = 10.0;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.sensorStatus.adxl_ok = false;
    OutputData.sensorStatus.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);

    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
    EXPECT_EQ(peakAltitude, 14);
}

TEST(StateMachineTest, LSMIdleToAscendFailMagnitudeDropFail){
    OutputData_t OutputData = {0};
    MagnitudeData_t MagnitudeData = {0};
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.sensorStatus.adxl_ok = false;
    OutputData.sensorStatus.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    MagnitudeData.lsm_accel_magnitude = 10;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, IdleToAscendOkAdxlFailMidwayLsmOkFull){
    OutputData_t OutputData = {0};
    MagnitudeData_t MagnitudeData = {0};
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.sensorStatus.adxl_ok = true;
    OutputData.sensorStatus.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.sensorStatus.adxl_ok = false;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, IdleToAscendFailAdxlFailMidwayLsmFailMidway){
    OutputData_t OutputData = {0};
    MagnitudeData_t MagnitudeData = {0};
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.sensorStatus.adxl_ok = true;
    OutputData.sensorStatus.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.sensorStatus.adxl_ok = false;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.sensorStatus.lsm_ok = false;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, IdleToAscendOkAdxlFailMidwayLsmMagnitudeSpike){
    OutputData_t OutputData = {0};
    MagnitudeData_t MagnitudeData = {0};
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.sensorStatus.adxl_ok = true;
    OutputData.sensorStatus.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.sensorStatus.adxl_ok = false;
    MagnitudeData.lsm_accel_magnitude = 3000;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    MagnitudeData.lsm_accel_magnitude = 20;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, IdleToAscendFailAdxlFailMidwayLsmMagnitudeSpike){
    OutputData_t OutputData = {0};
    MagnitudeData_t MagnitudeData = {0};
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.sensorStatus.adxl_ok = true;
    OutputData.sensorStatus.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.sensorStatus.adxl_ok = false;
    MagnitudeData.lsm_accel_magnitude = 3000;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    /// = 1 is failure because state change upon invalid data
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

/// ----------------------Apogee/ASCEND to DESCEND test------------------///
TEST(StateMachineTest, BMPUpdatePeakValidData){
    OutputData_t OutputData = {0};
    float mock_bmp_altitude[3] = {5,11,16.5}; 
    float mock_bmp_delta_alt[3] = {5,6,5.5};
    float bmp_peak_altitude = 0;
    uint8_t counter_state_change = 0;
    OutputData.sensorStatus.bmp_ok = true;
    for (int i = 0; i < 3; i++) {
        OutputData.bmp_alt = mock_bmp_altitude[i];
        AscendToDescend(OutputData, mock_bmp_delta_alt[i], bmp_peak_altitude, counter_state_change);
    }
    EXPECT_EQ(bmp_peak_altitude, (float)16.5);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, BMPUpdatePeakInvalidData){
    OutputData_t OutputData = {0};
    float mock_bmp_altitude[3] = {5,11,21.8}; 
    float mock_bmp_delta_alt[3] = {5,6,10.8};
    float bmp_peak_altitude = 0;
    uint8_t counter_state_change = 0;
    OutputData.sensorStatus.bmp_ok = true;
    for (int i = 0; i < 3; i++) {
        OutputData.bmp_alt = mock_bmp_altitude[i];
        AscendToDescend(OutputData, mock_bmp_delta_alt[i], bmp_peak_altitude, counter_state_change);
    }
    EXPECT_EQ(bmp_peak_altitude, (float)11);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, BMPStateChangeOkValidData){
    OutputData_t OutputData = {0};
    float mock_bmp_altitude[15] = {10,20,30,40,50,60,70,80,90,95,85,75,65,55,50}; 
    float mock_bmp_delta_alt[15] = {10,10,10,10,10,10,10,10,10,5,-10,-10,-10,-10,-5};
    float bmp_peak_altitude = 0;
    uint8_t counter_state_change = 0;
    OutputData.sensorStatus.bmp_ok = true;
    for (int i = 0; i < 15; i++) {
        OutputData.bmp_alt = mock_bmp_altitude[i];
        AscendToDescend(OutputData, mock_bmp_delta_alt[i], bmp_peak_altitude, counter_state_change);
    }
    EXPECT_EQ(OutputData.bmp_apogee_record, (float)95);
    EXPECT_EQ(OutputData.flightState, 2);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, BMPStateChangeFailInvalidDataDrop){
    OutputData_t OutputData = {0};
    float mock_bmp_altitude[17] = {10,20,30,40,50,60,70,80,90,95,85,75,65,55,100,105}; 
    float mock_bmp_delta_alt[17] = {10,10,10,10,10,10,10,10,10,5,-10,-10,-10,-10,45,5};
    float bmp_peak_altitude = 0;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 1;
    OutputData.sensorStatus.bmp_ok = true;
    for (int i = 0; i < 17; i++) {
        OutputData.bmp_alt = mock_bmp_altitude[i];
        AscendToDescend(OutputData, mock_bmp_delta_alt[i], bmp_peak_altitude, counter_state_change);
    }
    EXPECT_EQ(OutputData.bmp_apogee_record, (float)0);
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}