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

#ifndef HAWKEYE_UTILITIES_H
#define HAWKEYE_UTILITIES_H

#ifdef __cplusplus
extern "C" {
#endif


#include "scr_hawkeye.h"

// updates a ring buffer of floats
void HWK_UTIL_updateRingBuffer(float buffer[], const uint8_t size, uint8_t* i, float newValue);

// unwraps a ring buffer in order to an output array
void HWK_UTIL_unwrapRingBuffer(float inputBuffer[], float outputBuffer[], const uint8_t size, const uint8_t head);

#ifdef __cplusplus
}
#endif

#endif
