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

#include "scr_hawkeye.h"
#define HAWKEYE_DBL_EPSILON (1E-12) // some tests hang using standard double epsilon

// global vectors for test
hawkeye_vector3_t a = {
    .x = 4.96756,
    .y = 7.21934,
    .z = 9.12390,
};

hawkeye_vector3_t b = {
    .x = 9.12987,
    .y = 8.12349,
    .z = 1.23895,
};

// test implementations!
void vector3_add(void **state)
{
    (void)state;

    int retval;
    hawkeye_vector3_t r = {0};

    // test nullpointer cases
    retval = hawkeye_mathutil_vector3_add(NULL, &b, &r);
    assert_int_equal(retval, 1);
    retval = hawkeye_mathutil_vector3_add(&a, NULL, &r);
    assert_int_equal(retval, 1);
    retval = hawkeye_mathutil_vector3_add(&a, &b, NULL);
    assert_int_equal(retval, 1);

    // test nominal add case
    retval = hawkeye_mathutil_vector3_add(&a, &b, &r);
    assert_int_equal(retval, 0);
    assert_double_equal(r.x, 14.09743, HAWKEYE_DBL_EPSILON);
    assert_double_equal(r.y, 15.34283, HAWKEYE_DBL_EPSILON);
    assert_double_equal(r.z, 10.36285, HAWKEYE_DBL_EPSILON);
}

void vector3_sub(void **state)
{
    (void)state;

    int retval = 0;
    hawkeye_vector3_t r = {0};

    // test nullptr cases
    retval = hawkeye_mathutil_vector3_sub(NULL, &b, &r);
    assert_int_equal(retval, 1);
    retval = hawkeye_mathutil_vector3_sub(&a, NULL, &r);
    assert_int_equal(retval, 1);
    retval = hawkeye_mathutil_vector3_sub(&a, &b, NULL);
    assert_int_equal(retval, 1);

    // test nominal case
    retval = hawkeye_mathutil_vector3_sub(&a, &b, &r);
    assert_int_equal(retval, 0);
    assert_double_equal(r.x, -4.16231, HAWKEYE_DBL_EPSILON);
    assert_double_equal(r.y, -0.90415, HAWKEYE_DBL_EPSILON);
    assert_double_equal(r.z, 7.88495, HAWKEYE_DBL_EPSILON);
}
