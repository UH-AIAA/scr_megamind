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

#ifndef SCR_HAWKEYE_H
#define SCR_HAWKEYE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <math.h>
#include <stdint.h>

typedef struct {
    double x, y, z;
} hawkeye_vector3_t;

//////////////////////////////////////////////////////////////////
//                        MATH UTILITIES                        //
//////////////////////////////////////////////////////////////////

/////////////
/* VECTORS */
/////////////

// a + b, output stored in pointer r
int hawkeye_mathutil_vector3_add(hawkeye_vector3_t* a, hawkeye_vector3_t* b, hawkeye_vector3_t* r);

// a - b, output stored in pointer r
int hawkeye_mathutil_vector3_sub(hawkeye_vector3_t* a, hawkeye_vector3_t* b, hawkeye_vector3_t* r);

// || a ||, output stored in pointer r
int hawkeye_mathutil_vector3_norm(hawkeye_vector3_t* a, hawkeye_vector3_t* r);

// a ⋅ b, output stored in pointer r
int hawkeye_mathutil_vector3_dotp(hawkeye_vector3_t* a, hawkeye_vector3_t* b, double* r);

// a x b, output stored in pointer r
int hawkeye_mathutil_vector3_xpro(hawkeye_vector3_t* a, hawkeye_vector3_t* b, hawkeye_vector3_t* r);

// angle between vectors (rad), output stored in pointer r
int hawkeye_mathutil_vector3_theta(hawkeye_vector3_t* a, hawkeye_vector3_t* b, double* r);

// TODO: finish this LOL
/**
 * NAME: HWK_MATH_quadInterp
 * DESC: performs quadratic interpolation as a list of points
 * RET: 0 on success, one on error
 * ARGS: float x[] [IN]:           x values of your input points
 *       float y[] [IN]:           y values of your input points
 *       const uint8_t xSize [IN]: size of x value array
 *       const uint8_t ySize [IN]: size of y value array
 *       float* a [OUT]:           a coefficient
 *       float* b [OUT]:           b coefficient
 *       float* c [OUT]:           c coefficient
 *
 * NOTES: assumes y = ax^2 + bx + c
 *        assumes buffers are unwrapped (see HWK_UTIL_ringBuffer*)
 */
int HWK_MATH_quadInterp(float x[],
                               float y[],
                               const uint8_t xSize,
                               const uint8_t ySize,
                               float* a,
                               float* b,
                               float* c);


#ifdef __cplusplus
}
#endif

#endif
