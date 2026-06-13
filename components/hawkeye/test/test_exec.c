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

/* define your test function names here */

// hawkeye_mathutil
void vector3_add(void **state);
void vector3_sub(void **state);

// hawkeye_filters_simple
void FILT_lowPass(void **state);
void FILT_Welford(void **state);

// hawkeye_utilities
void UTIL_updateRingBuffer(void **state);
void UTIL_unwrapRingBuffer(void **state);

// hawkeye_fsm
void FSM_launchAccel(void **state);
void FSM_apogeeAltVel(void **state);
void FSM_landingAlt(void **state);

// entry point
int main(int argc, char* argv[]) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(vector3_add),
        cmocka_unit_test(vector3_sub),
        cmocka_unit_test(FILT_lowPass),
        cmocka_unit_test(FILT_Welford),
        cmocka_unit_test(UTIL_updateRingBuffer),
        cmocka_unit_test(UTIL_unwrapRingBuffer),
        cmocka_unit_test(FSM_launchAccel),
        cmocka_unit_test(FSM_apogeeAltVel),
        cmocka_unit_test(FSM_landingAlt),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
