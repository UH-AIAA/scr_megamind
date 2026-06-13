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

#include "hawkeye_fsm.h"
#include "hawkeye_filters_simple.h"
#include <float.h>
#include <stdint.h>


/**
 * NAME: HWK_FSM_launchDetectAccel
 * AUTH: Nathan Samuell
 * DESC: detects launch using acceleration as input
 * RET:  FSM_NOOP if state is unchanged, FSM_UPDATE
 *       if conditions met
 * ARGS: float accelMag [IN]:          current magnitude of acceleration
 *       uint8_t* counter [IN/OUT]:    location of condition counter
 *       const uint8_t reqCount [IN]:  successes until state change
 *       const float accelThresh [IN]: threshold for launch accel
 * NOTES: you MUST manage the data lifetime of counter on your own.
 *        Hawkeye aims to minimize any kind of internal or obscured
 *        behavior. reqCount/accelThresh is intended to be populated
 *        with a #define (or constexpr if using C++), but was not set
 *        to a default to avoid making the developer call #unset to change.
 */
FSM_Err_t HWK_FSM_launchDetectAccel(float accelMag,
                                    uint8_t* counter,
                                    const uint8_t reqCount,
                                    const float accelThresh)
{
    // nullptr check
    if (counter == NULL)
    {
        return FSM_ERR_NULLPTR;
    }

    // are we over our threshold?
    if (accelMag >= accelThresh)
    {
        *counter += 1;
    }
    // if not, noise, reset
    else
    {
        *counter = 0;

        // returns in case dev left
        // reqCount as zero so it won't
        // automatically fall to next state
        return FSM_NOOP;
    }

    // have we hit enough times?
    if(*counter == reqCount)
    {
        *counter = 0;
        return FSM_UPDATE;
    }

    return FSM_NOOP;
}


FSM_Err_t HWK_FSM_apogeeDetectAltVel(float alt,
                                     uint64_t time,
                                     uint8_t* counter,
                                     const uint8_t reqCount,
                                     const float negVelThresh)
{
    // nullptr check
    if (counter == NULL)
    {
        return FSM_ERR_NULLPTR;
    }

    // for now, just use prev and curr altitude. implement a ring buffer next
    static float prevAlt;
    static float currAlt;
    static uint64_t prevTime;
    static uint64_t currTime;

    // step forward
    prevAlt = currAlt;
    currAlt = alt;
    prevTime = currTime;
    currTime = time;

    // if we have increasing altitude, apogee not even eligible
    if(currAlt > prevAlt)
    {
        *counter = 0;
        return FSM_NOOP;
    }

    // now, calculate vertical velocity using time step
    uint64_t dt = currTime - prevTime;
    float vel = (currAlt - prevAlt) / dt;

    /**
     * NOTE: calculating velocity this way is very noisy. whatever
     * noise is present in your altitude will be heavily amplified.
     * it is strongly recommended to filter your measurements
     * BEFORE you put them into this function (see HWK_FILW_lowPass())
     *
     * we don't reset the counter since if velocity is not negative
     * since we already did an eligibility check with the altitude
     * difference. why trust the more noisy measurement?
     */
    // if velocity is negative enough,
    if (vel < negVelThresh)
    {
        // update counter
        *counter += 1;
    }

    // if we have enough negative velocities...
    if(*counter == reqCount)
    {
        *counter = 0;
        return FSM_UPDATE;
    }

    return FSM_NOOP;
}


FSM_Err_t HWK_FSM_landingDetectAlt(float alt,
                                   uint8_t* counter,
                                   const uint8_t reqCount,
                                   const float divThr)
{
    // nullptr check
    if (counter == NULL)
    {
        return FSM_ERR_NULLPTR;
    }

    // var init
    static float altMean = 0;  // mean altitude
    static float altVariance = 0;  // variance in mean altitude
    static float welfMU = 0;   // mediary unit for welford
    static uint32_t n = 0;  // num iterations

    // run Welford's on altitude
    FILT_Err_t wStatus = HWK_FILT_Welford(alt, &altMean, &altVariance, &welfMU, &n);

    // check status
    if (wStatus != FILT_SUCCESS)
    {
        // reset filter
        altMean = 0;
        altVariance = 0;
        welfMU = 0;
        n = 0;
        *counter = 0;

        return FSM_NOOP;
    }

    // are we over our variance?
    if (altVariance >= divThr)
    {
        // reset filter
        altMean = 0;
        altVariance = 0;
        welfMU = 0;
        n = 0;
        *counter = 0;

        return FSM_NOOP;
    }

    // update count
    if(n > 1)
    {
        // only update if it's not our first,
        // variance is always zero on first run
        *counter += 1;
    }

    // are we done?
    if (*counter == reqCount)
    {
        *counter = 0;
        return FSM_UPDATE;
    }

    return FSM_NOOP;
}
