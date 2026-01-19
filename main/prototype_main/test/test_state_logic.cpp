#include <gtest/gtest.h>
#include "../include/prototype_main.h"
#include "../constants/Const.h"

TEST(StateMachineTest, AdxlIdleToDescendOk) {
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    OutputData.bmp_alt = 10.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    OutputData.adxl_ok = true;

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
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    OutputData.adxl_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    MagnitudeData.adxl_accel_magnitude = 10;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, LsmIdleToDescendOk) {
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    OutputData.bmp_alt = 10.0;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.adxl_ok = false;
    OutputData.lsm_ok = true;

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
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.adxl_ok = false;
    OutputData.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    MagnitudeData.lsm_accel_magnitude = 10;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, IdleToAscendOkAdxlFailMidwayLsmOkFull){
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.adxl_ok = true;
    OutputData.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.adxl_ok = false;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, IdleToAscendFailAdxlFailMidwayLsmFailMidway){
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.adxl_ok = true;
    OutputData.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.adxl_ok = false;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.lsm_ok = false;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, IdleToAscendOkAdxlFailMidwayLsmMagnitudeSpike){
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.adxl_ok = true;
    OutputData.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.adxl_ok = false;
    MagnitudeData.lsm_accel_magnitude = 3000;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    MagnitudeData.lsm_accel_magnitude = 20;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

TEST(StateMachineTest, IdleToAscendFailAdxlFailMidwayLsmMagnitudeSpike){
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 20;
    MagnitudeData.lsm_accel_magnitude = 20;
    OutputData.adxl_ok = true;
    OutputData.lsm_ok = true;

    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.adxl_ok = false;
    MagnitudeData.lsm_accel_magnitude = 3000;
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    IdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    /// = 1 is failure because state change upon invalid data
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
}

