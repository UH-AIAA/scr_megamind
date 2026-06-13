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

#include "scr_hawkeye.h"

int hawkeye_mathutil_vector3_add(hawkeye_vector3_t *a, hawkeye_vector3_t *b, hawkeye_vector3_t *r) {
    if(a == NULL || b == NULL || r == NULL) {
        return 1;
    }
    r->x = a->x + b->x;
    r->y = a->y + b->y;
    r->z = a->z + b->z;

    return 0;
}

int hawkeye_mathutil_vector3_sub(hawkeye_vector3_t* a, hawkeye_vector3_t* b, hawkeye_vector3_t* r) {
    if(a == NULL || b == NULL || r == NULL) {
        return 1;
    }
    r->x = a->x - b->x;
    r->y = a->y - b->y;
    r->z = a->z - b->z;

    return 0;
}

int hawkeye_mathutil_vector3_mul(hawkeye_vector3_t* a, hawkeye_vector3_t* b, hawkeye_vector3_t* r) {
    if(a == NULL || b == NULL || r == NULL) {
        return 1;
    }
    r->x = a->x * b->x;
    r->y = a->y * b->y;
    r->z = a->z * b->z;

    return 0;
}

int hawkeye_mathutil_vector3_div(hawkeye_vector3_t* a, hawkeye_vector3_t* b, hawkeye_vector3_t* r) {
    if(a == NULL || b == NULL || r == NULL) {
        return 1;
    }
    if(b->x == 0 || b->y == 0 || b->z == 0) {
        return 2;
    }
    r->x = a->x / b->x;
    r->y = a->y / b->y;
    r->z = a->z / b->z;

    return 0;
}

// normalize!
// v / ||v|| (v divided by the magnitude of v)
int hawkeye_mathutil_vector3_norm(hawkeye_vector3_t* v, hawkeye_vector3_t* u) {
    if(v == NULL || u == NULL) {
        return 1;
    }

    float mag_v = sqrtf(    // get magnitude!
        (v->x * v->x) +
        (v->y * v->y) +
        (v->z * v->z) );

    u->x = v->x / mag_v;    // divide x component
    u->y = v->y / mag_v;    // divide y component
    u->z = v->z / mag_v;    // divide z component

    return 0;
}

// dot product
int hawkeye_mathutil_vector3_dotp(hawkeye_vector3_t* a, hawkeye_vector3_t* b, double* r) {
    if(a == NULL || b == NULL || r == NULL) {
        return 1;
    }

    // compute dot product, assign to result
    *r = (a->x * b->x) + (a->y * b->y) + (a->z * b->z);

    return 0;
}

// cross product
int hawkeye_mathutil_vector3_xpro(hawkeye_vector3_t* a, hawkeye_vector3_t* b, hawkeye_vector3_t* r) {
    if(a == NULL || b == NULL || r == NULL) {
        return 1;
    }

    r->x = (a->y * b->z) - (a->z * b->y);   // first determinant
    r->y = (a->z * b->x) - (a->x * b->z);   // second determinant
    r->z = (a->x * b->y) - (a->y * b->x);   // third determinant

    return 0;
}

// angle between vectors
// dot product = ||a||*||b||*cos(theta)
// uses RADIANS
int hawkeye_mathutil_vector3_theta(hawkeye_vector3_t* a, hawkeye_vector3_t* b, double* r) {
    if(a == NULL || b == NULL || r == NULL) {
        return 1;
    }

    // setup variables
    double a_mag, b_mag, dot_product;

    // get cross product and magnitudes
    hawkeye_mathutil_vector3_dotp(a, b, &dot_product);
    a_mag = sqrtf(
        (a->x * a->x) +
        (a->y * a->y) +
        (a->z + a->z) );
    b_mag = sqrtf(
        (b->x * b->x) +
        (b->y * b->y) +
        (b->z + b->z) );

    // compute theta
    *r = acosf(dot_product / (a_mag * b_mag));

    return 0;
}
