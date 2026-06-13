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
#include "hawkeye_utilities.h"

#include <string.h>

void UTIL_updateRingBuffer(void **state)
{
    (void)state;

    // test setup
    const uint8_t size = 5;
    uint8_t i = 0;
    float buf[5] = {0, 0, 0, 0, 0};
    float singleAddAnswer[5] = {8.28, 0, 0, 0, 0};
    float wrapAroundAnswer[5] = {5, 6, 2, 3, 4};

    // test single add
    HWK_UTIL_updateRingBuffer(buf, size, &i, 8.28);
    assert_int_equal(i, 1);
    assert_memory_equal(buf, singleAddAnswer, sizeof(buf));
    memset(&buf, 0, sizeof(buf));   // teardown
    i = 0;

    // test wraparound adding
    for(int ii = 0; ii < 7; ii++)
    {
        HWK_UTIL_updateRingBuffer(buf, size, &i, ii);
    }

    assert_int_equal(i, 2);
    assert_memory_equal(buf, wrapAroundAnswer, sizeof(buf));
}


void UTIL_unwrapRingBuffer(void **state)
{
    (void)state;

    // setup
    float buf[5] = {10, 11, 12, 13, 9};
    float ans[5] = {9, 10, 11, 12, 13};
    float out[5] = {0};
    const uint8_t size = 5;
    uint8_t head = 4;

    // test unwrap
    HWK_UTIL_unwrapRingBuffer(buf, out, size, head);
    assert_memory_equal(out, ans, sizeof(out));
}
