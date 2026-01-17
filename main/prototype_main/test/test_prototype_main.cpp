#include <gtest/gtest.h>
#include "../include/prototype_main.h"
#include "../constants/Const.h"

//Write unit test for calibration proecess with mock data

TEST(StateMachineTest, ADXLdleToDescendOk) {
    OutputData_t gOutputData;
    MagnitudeData_t gMagnitudeData;
    uint8_t counter_state_change = 0;
    gOutputData.flightState = 0;
    float peakAltitude = 0.0;
    gOutputData.bmp_alt = 10.0;
    gMagnitudeData.adxl_accel_magnitude = 20;

    ADXLIdleToAscend(gOutputData, gMagnitudeData, counter_state_change, peakAltitude);
    gOutputData.bmp_alt++;
    ADXLIdleToAscend(gOutputData, gMagnitudeData, counter_state_change, peakAltitude);
    gOutputData.bmp_alt++;
    ADXLIdleToAscend(gOutputData, gMagnitudeData, counter_state_change, peakAltitude);
    
    EXPECT_EQ(gOutputData.flightState, 1);
    EXPECT_EQ(counter_state_change, 0);
    EXPECT_EQ(peakAltitude, 12);
}

TEST(StateMachineTest, ADXLIdleToAscendFail){
    OutputData_t gOutputData;
    MagnitudeData_t gMagnitudeData;
    uint8_t counter_state_change = 2;
    gOutputData.flightState = 0;
    float peakAltitude = 0.0;
    gMagnitudeData.adxl_accel_magnitude = 10;

    ADXLIdleToAscend(gOutputData, gMagnitudeData, counter_state_change, peakAltitude);
    gMagnitudeData.adxl_accel_magnitude = 20;
    ADXLIdleToAscend(gOutputData, gMagnitudeData, counter_state_change, peakAltitude);
    ADXLIdleToAscend(gOutputData, gMagnitudeData, counter_state_change, peakAltitude);
    EXPECT_NE(gOutputData.flightState, 1);
}