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

#include "hawkeye_utilities.h"

/**
 * NAME: HWK_UTIL_updateRingBuffer
 * AUTH: Nathan Samuell
 * DESC: updates a ring buffer of floats
 * RET:  None
 * ARGS: float buffer[] [IN]:     raw input value
 *       const uint8_t size [IN]: size of the input array
 *       uint8_t* i [IN]:         current ring buffer index
 *       float newValue [IN]:     value to add to ring buffer.
 * NOTES: i index must be tracked outside of the function
 */
void HWK_UTIL_updateRingBuffer(float buffer[], const uint8_t size, uint8_t* i, float newValue)
{
    // update value
    buffer[*i] = newValue;
    *i = (*i + 1) % size;   // modulus means index will wrap around when buffer is full

    return;
}

/**
 * NAME: HWK_UTIL_unwrapRingBuffer
 * AUTH: Nathan Samuell
 * DESC: unwraps a ring buffer into an output array
 * RET:  None
 * ARGS: float inputBuffer[] [IN]:   input buffer
 *       float outputBuffer[] [OUT]: output buffer
 *       const uint8_t size [IN]:    size of the input array
 *       const uint8_t head [IN]:    current head of ring buffer
 * NOTES: head is essentially the i value of the update function
 */
void HWK_UTIL_unwrapRingBuffer(float inputBuffer[], float outputBuffer[], const uint8_t size, const uint8_t head)
{
    for (int i = 0; i < size; i++)
    {
        outputBuffer[i] = inputBuffer[(head + i) % size];
    }

    return;
}

// update, added it anyways, but use this later on
/**  not adding an unwrap function, use the following pattern:
 * for(uint8_t i = head; i < size; i++)
 * {
 *     process(ring[i]);
 * }
 *
 * for(uint8_t i = 0; i < head; i++)
 * {
 *     process(ring[i]);
 * }
**/
