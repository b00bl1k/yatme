/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Alexey Ryabov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stddef.h>
#include <cayenne_lpp.h>
#include "board.h"
#include "owbus.h"
#include "ds18b20.h"
#include "radio.h"

cayenne_lpp_t msg;

int main()
{
    int status;
    uint16_t value;

    board_init();
    radio_init();
    owbus_init();

    status = ds18b20_convert_t(NULL);

    value = board_vbat_read();
    cayenne_lpp_add_analog_input(&msg, 1, (float)value / 100.0);

    if (status == 0) {
        board_delay_us(800000);
        status = ds18b20_read_t(NULL, &value);
        if (status == 0) {
            cayenne_lpp_add_temperature(&msg, 2,
                (float)DS18B20_NORMALIZE(value) / 10.0);
        }
    }

    radio_tx(msg.buffer, msg.cursor);
    board_sleep();

    for (;;) {
    }

    return 0;
}
