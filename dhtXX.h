/*
 * MIT License
 * 
 * Copyright (c) 2018-2021 Michele Biondi, Ernst-Georg Schmid
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#ifndef DHTXX_H_
#define DHTXX_H_

#include "driver/gpio.h"

enum dhtXX_type
{
    DHT11 = -1,
    DHT22
};

enum dhtXX_status
{
    DHTXX_MIN_FREQ_ERROR = -3,
    DHTXX_CRC_ERROR,
    DHTXX_TIMEOUT_ERROR,
    DHTXX_OK
};

struct dhtXX_reading
{
    int status;
    int temperature_integer;
    int temperature_fractional;
    int humidity_integer;
    int humidity_fractional;
    int sensor_type;
    bool in_range;
};

void dhtXX_init(gpio_num_t, int);

struct dhtXX_reading dhtXX_read();

#endif