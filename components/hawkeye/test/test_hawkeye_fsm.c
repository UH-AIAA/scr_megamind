/*
 * scr_Hawkeye - a sounding/hobby rocket GNC library written in C
 * Copyright (C) 2025-2026 Nathan Samuell
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/


#include <cmocka.h>
#include <hawkeye_fsm.h>

void FSM_launchAccel(void **state)
{
    (void)state;

    uint8_t counter = 0;
    const uint8_t reqCount = 5;
    const float accelThresh = 50; // ~50 Gs

    FSM_Err_t status = FSM_NOOP;

    // check nullptr
    status = HWK_FSM_launchDetectAccel(0.0, NULL, reqCount, accelThresh);
    assert_int_equal(status, FSM_ERR_NULLPTR);

    // check turnaround/reset behavior
    float turnAroundAccel[5] = {55, 97, 30, 100, 100};
    uint8_t turnAroundCounts[5] = {1, 2, 0, 1, 2};

    for(int i = 0; i < 5; i++)
    {
        status = HWK_FSM_launchDetectAccel(turnAroundAccel[i], &counter, reqCount, accelThresh);
        assert_int_equal(status, FSM_NOOP);
        assert_int_equal(counter, turnAroundCounts[i]);
    }

    // check nominal behavior
    counter = 0;
    float nominalAccel[12] = {0, 0, 3, 0, 3, 0, 20, 55, 70, 70, 70, 90};
    uint8_t nominalCounts[12] = {0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 0};
    FSM_Err_t nominalStates[12] = {FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_UPDATE};

    for(int i = 0; i < 12; i++)
    {
        status = HWK_FSM_launchDetectAccel(nominalAccel[i], &counter, reqCount, accelThresh);
        assert_int_equal(status, nominalStates[i]);
        assert_int_equal(counter, nominalCounts[i]);
    }

}

void FSM_apogeeAltVel(void **state)
{
    (void)state;

    uint8_t counter = 0;
    const uint8_t reqCount = 5;
    const float negVelThresh = -1.5;

    FSM_Err_t status = FSM_NOOP;

    // nullptr check
    status = HWK_FSM_apogeeDetectAltVel(0.0, 0, NULL, reqCount, negVelThresh);
    assert_int_equal(status, FSM_ERR_NULLPTR);

    // check hard rejection behavior
    float rejectAlts[5] = {200, 190, 210, 200, 190};
    uint64_t rejectTimes[5] = {1, 2, 3, 4, 5};
    uint8_t rejectCounts[5] = {0, 1, 0, 1, 2};

    for(int i = 0; i < 5; i++)
    {
        status = HWK_FSM_apogeeDetectAltVel(rejectAlts[i], rejectTimes[i], &counter, reqCount, negVelThresh);
        assert_int_equal(status, FSM_NOOP);
        assert_int_equal(counter, rejectCounts[i]);
    }

    // check nominal behavior
    // NOTE: also checks soft failure behavior
    counter = 0;
    float nominalAlts[15] = {0, 1, 10, 100, 200, 9500, 9700, 9900, 10001, 10000, 9980, 9950, 9900, 9850, 9800};
    uint64_t nominalTimes[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    uint8_t nominalCounts[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 0};
    FSM_Err_t nominalStatus[15] = {FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_UPDATE};

    for(int i = 0; i < 15; i++)
    {
        status = HWK_FSM_apogeeDetectAltVel(nominalAlts[i], nominalTimes[i], &counter, reqCount, negVelThresh);
        assert_int_equal(status, nominalStatus[i]);
        assert_int_equal(counter, nominalCounts[i]);
    }
}

void FSM_landingAlt(void **state)
{
    (void)state;

    uint8_t counter = 0;
    const uint8_t reqCount = 20;
    const float divThr = 1.2;

    // nullptr check
    FSM_Err_t status = HWK_FSM_landingDetectAlt(0, NULL, reqCount, divThr);
    assert_int_equal(status, FSM_ERR_NULLPTR);

    // test rejection behavior
    float rejectionAlts[5] = {250, 240, 230, 220, 200};
    float rejectionCounts[5] = {0, 0, 0, 0, 0};

    for(int i = 0; i < 5; i++)
    {
        status = HWK_FSM_landingDetectAlt(rejectionAlts[i], &counter, reqCount, divThr);
        assert_int_equal(status, FSM_NOOP);
        assert_int_equal(counter, rejectionCounts[i]);
    }

    // test nominal behavior
    float nominalAlts[22] = {0.05, -0.01, 0.1, 0.002, -0.1, -0.2, -0.1, 0.2, -0.15, 0.04, 0.05, -0.01, 0.1, 0.002, -0.1, -0.2, -0.1, 0.2, -0.15, 0.04, 0, 0};
    float nominalCounts[22] = {0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 0};
    FSM_Err_t nominalStatus[22] = {FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_NOOP, FSM_UPDATE};

    for(int i = 0; i < 22; i++)
    {
        status = HWK_FSM_landingDetectAlt(nominalAlts[i], &counter, reqCount, divThr);
        assert_int_equal(counter, nominalCounts[i]);
        assert_int_equal(status, nominalStatus[i]);
    }
}
