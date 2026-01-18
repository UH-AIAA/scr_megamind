#include <gtest/gtest.h>
#include "../include/prototype_main.h"
#include "../constants/Const.h"

TEST(StateMachineTest, ADXLdleToDescendOk) {
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    OutputData.bmp_alt = 10.0;
    MagnitudeData.adxl_accel_magnitude = 20;

    ADXLIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    ADXLIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    ADXLIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
    EXPECT_EQ(peakAltitude, 12);
}

TEST(StateMachineTest, ADXLIdleToAscendFail){
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 2;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.adxl_accel_magnitude = 10;

    ADXLIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    MagnitudeData.adxl_accel_magnitude = 20;
    ADXLIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    ADXLIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(OutputData.flightState, 1);
}

TEST(StateMachineTest, LSMdleToDescendOk) {
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 0;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    OutputData.bmp_alt = 10.0;
    MagnitudeData.lsm_accel_magnitude = 20;

    LSMIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    LSMIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    OutputData.bmp_alt++;
    LSMIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    
    EXPECT_EQ(OutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
    EXPECT_EQ(peakAltitude, 12);
}

TEST(StateMachineTest, LSMIdleToAscendFail){
    OutputData_t OutputData;
    MagnitudeData_t MagnitudeData;
    uint8_t counter_state_change = 2;
    OutputData.flightState = 0;
    float peakAltitude = 0.0;
    MagnitudeData.lsm_accel_magnitude = 10;

    LSMIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    MagnitudeData.lsm_accel_magnitude = 20;
    LSMIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    LSMIdleToAscend(OutputData, MagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(OutputData.flightState, 1);
}

