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

#include "hawkeye_filters_simple.h"
#include <float.h>
#include <stdio.h>

/**
 * NAME: HWK_FILT_lowPass
 * AUTH: Nathan Samuell
 * DESC: performs weighted exponential moving average filtering on some value
 * RET:  0 on success
 * ARGS: float raw [IN]:           raw input value
 *       float* filtered [IN/OUT]: running total filtered value
 *       const float alpha [IN]:   alpha filter weight
 * NOTES: assumes that the input to filtered is kept track of elsewhere,
 *        this is an "online" algorithm, so the state of the filter is
 *        encapsulated in that variable
 *
 *        see link for more info:
 *        https://kleinembedded.com/quick-and-simple-digital-filters-for-smoothing-and-dc-removal/
 */
FILT_Err_t HWK_FILT_lowPass(float raw, float* filtered, const float alpha)
{
    // null pointer check
    if(filtered == NULL)
    {
        return FILT_ERR_NULLPTR;
    }
    // alpha must be between 0 and 1
    if(alpha < 0 || alpha > 1)
    {
        return FILT_ERR_INVALID_MULT;
    }

    // update lowpass IIR in place
    *filtered = alpha * raw + (1 - alpha) * (*filtered);

    // success
    return FILT_SUCCESS;
}

/**
 * NAME: HWK_FILT_Welford
 * AUTH: Nathan Samuell/Nathan Munischke
 * DESC: performs running average using Welford's online algorithm
 * RET:  see FILT_Err_t
 * ARGS: float measurement [IN]:      new measurement to add
 *       float* mean [IN/OUT]:        current online mean
 *       float* variance [IN/OUT]:    current online variance
 *       float* mediaryUnit [IN/OUT]: used in calculation
 *       uint32_t* n [IN/OUT]:        tracks iterations
 * NOTES: assumes that the input to filter is kept track of elsewhere,
 *        this is an "online" algorithm, so the state of the filter is
 *        encapsulated in the in/out variables
 */
FILT_Err_t HWK_FILT_Welford(float measurement, float* mean, float* variance, float* mediaryUnit, uint32_t* n)
{
    // nullptr check
    if(mean == NULL ||variance == NULL || mediaryUnit == NULL || n == NULL)
    {
        return FILT_ERR_NULLPTR;
    }

    // var init
    float weightingFactor = 0;  // weights our input

    // increment iterations
    *n += 1;

    // calculate new input weight
    weightingFactor = measurement - *mean;

    // online update mean
    *mean += weightingFactor / *n;

    // calculate mediary unit
    *mediaryUnit += weightingFactor * (measurement - *mean);

    // update variance
    if(*n > 1)
    {
        // should never be since this is an int,
        // but why not
        if(*n - 1 < FLT_EPSILON)
        {
            return FILT_ERR_DBZ;
        }

        *variance = *mediaryUnit / (*n - 1);
    }
    else
    {
        // no variance if we've only sampled once
        *variance = 0;
    }

    return FILT_SUCCESS;
}
