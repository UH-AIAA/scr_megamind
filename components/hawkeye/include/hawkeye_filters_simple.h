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


#ifndef HAWKEYE_FILTERS_SIMPLE_H
#define HAWKEYE_FILTERS_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "scr_hawkeye.h"

typedef enum {
    FILT_SUCCESS = 0,
    FILT_ERR_NULLPTR = 1,
    FILT_ERR_INVALID_MULT = 2,
    FILT_ERR_DBZ = 3,
} FILT_Err_t;

// simple online low-pass filter
FILT_Err_t HWK_FILT_lowPass(float raw, float* filtered, const float alpha);

// simple online running, unweighted average
FILT_Err_t HWK_FILT_Welford(float measurement, float* mean, float* variance, float* mediaryUnit, uint32_t* n);

#ifdef __cplusplus
}
#endif

#endif
