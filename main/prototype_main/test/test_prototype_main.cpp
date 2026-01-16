#include <gtest/gtest.h>
#include "prototype_main/include/prototype_main.h"

TEST(StateMachineTest, AdxldleToDescendOk) {
    OutputData_t gOutputData;
    MagnitudeData_t gMagnitudeData;
    gOutputData.flightState = 0;
    gOutputData.adxl_ok = true;
    gMagnitudeData.adxl_accel_magnitude = 20;
    Core0_stateMachine(nullptr);
    EXPECT_EQ(gOutputData.flightState, 0);
}