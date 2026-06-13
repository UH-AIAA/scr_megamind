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

#ifndef HAWKEYE_FSM_H
#define HAWKEYE_FSM_H

#ifdef __cplusplus
extern "C" {
#endif


#include "scr_hawkeye.h"

typedef enum {
    FSM_NOOP = 0,
    FSM_UPDATE = 1,
    FSM_ERR_NULLPTR = 2,
} FSM_Err_t;

// detects launch using acceleration
FSM_Err_t HWK_FSM_launchDetectAccel(float accelMag,
                                    uint8_t* counter,
                                    const uint8_t reqCount,
                                    const float accelThresh
);

// TODO:
// detects launch using altitude
FSM_Err_t HWK_FSM_launchDetectAlt(float alt,
                                  uint8_t* counter,
                                  const uint8_t reqCount
);

// detects apogee using alt and derived velocity
FSM_Err_t HWK_FSM_apogeeDetectAltVel(float alt,
                                     uint64_t time,
                                     uint8_t* counter,
                                     const uint8_t reqCount,
                                     const float negVelThresh);

// detects landing using altitude
FSM_Err_t HWK_FSM_landingDetectAlt(float alt,
                                   uint8_t* counter,
                                   const uint8_t reqCount,
                                   const float divThr);


#ifdef __cplusplus
}
#endif

#endif
