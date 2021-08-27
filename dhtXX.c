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

#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dhtXX.h"

static gpio_num_t dht_gpio;
static int sensor_type;
static int64_t last_read_time = -2000000;
static int64_t min_read_time;
static struct dhtXX_reading last_read;

static int _waitOrTimeout(uint16_t microSeconds, int level)
{
    int micros_ticks = 0;
    while (gpio_get_level(dht_gpio) == level)
    {
        if (micros_ticks++ > microSeconds)
            return DHTXX_TIMEOUT_ERROR;
        ets_delay_us(1);
    }
    return micros_ticks;
}

static int _checkCRC(uint8_t data[])
{
    //printf("%d=%d\n", (uint8_t) (data[0] + data[1] + data[2] + data[3]), data[4]);
    if (data[4] == (uint8_t) (data[0] + data[1] + data[2] + data[3]))
        return DHTXX_OK;
    else
        return DHTXX_CRC_ERROR;
}

static void _sendStartSignal()
{
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_gpio, 0);
    ets_delay_us(20 * 1000);
    gpio_set_level(dht_gpio, 1);
    ets_delay_us(40);
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
}

static int _checkResponse()
{
    /* Wait for next step ~80us*/
    if (_waitOrTimeout(80, 0) == DHTXX_TIMEOUT_ERROR)
        return DHTXX_TIMEOUT_ERROR;

    /* Wait for next step ~80us*/
    if (_waitOrTimeout(80, 1) == DHTXX_TIMEOUT_ERROR)
        return DHTXX_TIMEOUT_ERROR;

    return DHTXX_OK;
}

static struct dhtXX_reading _minFreqError()
{
    struct dhtXX_reading minFreqError = {DHTXX_MIN_FREQ_ERROR, -1, -1, -1, -1, -1, false};
    return minFreqError;
}

static struct dhtXX_reading _timeoutError()
{
    struct dhtXX_reading timeoutError = {DHTXX_TIMEOUT_ERROR, -1, -1, -1, -1, -1, false};
    return timeoutError;
}

static struct dhtXX_reading _crcError()
{
    struct dhtXX_reading crcError = {DHTXX_CRC_ERROR, -1, -1, -1, -1, -1, false};
    return crcError;
}

void dhtXX_init(gpio_num_t gpio_num, int type)
{
    /* Wait 2 seconds to make the device pass its initial unstable status */
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    dht_gpio = gpio_num;
    sensor_type = type;

    if (sensor_type == DHT11)
    {
        min_read_time = 1000000;
    }
    else
    {
        min_read_time = 2000000;
    }

    //printf("%d\n", sensor_type);
}

struct dhtXX_reading dhtXX_read()
{

    /* Try to sense if sampling frequency is too high (DHT11 min. 1 Hz, DHT22 min. 0.5 Hz) */

    if (esp_timer_get_time() - min_read_time < last_read_time)
    {
        return last_read = _minFreqError();
    }

    last_read_time = esp_timer_get_time();

    uint8_t data[5] = {0x0, 0x0, 0x0, 0x0, 0x0};

    _sendStartSignal();

    if (_checkResponse() == DHTXX_TIMEOUT_ERROR)
    {
        return last_read = _timeoutError();
    }

    /* Read response */
    for (int i = 0; i < 40; i++)
    {
        /* Initial data */
        if (_waitOrTimeout(50, 0) == DHTXX_TIMEOUT_ERROR)
        {
            return last_read = _timeoutError();
        }

        if (_waitOrTimeout(70, 1) > 28)
        {
            /* Bit received was a 1 */
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    /*for (int i = 0; i < 5; i++)
    {
        printf("Data %d = %d\n", i, data[i]);
    }*/

    if (_checkCRC(data) != DHTXX_CRC_ERROR)
    {
        last_read.status = DHTXX_OK;
        if (sensor_type == DHT11)
        {
            last_read.humidity_integer = data[0];
            last_read.humidity_fractional = data[1];
            last_read.temperature_integer = data[2];
            last_read.temperature_fractional = data[3];
        }
        else
        {
            int16_t h = (data[0] << 8) + data[1];
            int16_t t = (data[2] << 8) + data[3];
            
            last_read.humidity_integer = h / 10;
            last_read.humidity_fractional = h % 10;
            last_read.temperature_integer = t / 10;
            last_read.temperature_fractional = t % 10;
        }
        last_read.sensor_type = sensor_type;
        last_read.in_range = false;

        if (sensor_type == DHT11 && last_read.temperature_integer >= 0 && last_read.temperature_integer <= 50 && last_read.humidity_integer >= 20 && last_read.humidity_integer <= 80)
        {
            last_read.in_range = true;
        }
        else if ((sensor_type == DHT22 && last_read.temperature_integer >= -40 && last_read.temperature_integer <= 80 && last_read.humidity_integer >= 0 && last_read.humidity_integer <= 100))
        {
            last_read.in_range = true;
        }

        return last_read;
    }
    else
    {
        return last_read = _crcError();
    }
}
