#include <gtest/gtest.h>
#include "../include/prototype_main.h"
#include "../constants/Const.h"

TEST(DataCalibrationTest, ADXLCalibration) {
    ADXLCalibrate_t ADXLCalibrateVars;
    MagnitudeData_t MagnitudeData;
    OutputData_t OutputData;
    float adxl_mock_acc_x[21] = {0,1,2,1,0,1,2,1,0,1,
                                 0,1,2,1,0,1,2,1,0,0,0};
    float adxl_mock_acc_y[21] = {0,1,2,1,0,1,2,1,0,1,
                                 0,1,2,1,0,1,2,1,0,1,1};
    float adxl_mock_acc_z[21] = {0,1,2,1,0,1,2,1,0,1,
                                 0,1,2,1,0,1,2,1,0,2,2};
    for(int i = 0; i <= 20; i++) {
        OutputData.adxl_acc_x = adxl_mock_acc_x[i];
        OutputData.adxl_acc_y = adxl_mock_acc_y[i];
        OutputData.adxl_acc_z = adxl_mock_acc_z[i];
        ADXLCalibrate(ADXLCalibrateVars, MagnitudeData, OutputData);
    }

    EXPECT_EQ(ADXLCalibrateVars.adxl_accel_x_mean, (float)0.85);
    EXPECT_EQ(ADXLCalibrateVars.adxl_accel_y_mean, (float)0.9);
    EXPECT_EQ(ADXLCalibrateVars.adxl_accel_z_mean, (float)0.95);

    ADXLCalibrate(ADXLCalibrateVars, MagnitudeData, OutputData);
    EXPECT_EQ(OutputData.adxl_acc_x, (float)-0.85);
    EXPECT_EQ((std::round((OutputData.adxl_acc_y * 10))) / 10, (float)0.1);
    EXPECT_EQ(OutputData.adxl_acc_z, (float)1.05);
    EXPECT_EQ(MagnitudeData.adxl_accel_magnitude, sqrtf(OutputData.adxl_acc_x*OutputData.adxl_acc_x + 
                                                        OutputData.adxl_acc_y*OutputData.adxl_acc_y + 
                                                        OutputData.adxl_acc_z*OutputData.adxl_acc_z));

}