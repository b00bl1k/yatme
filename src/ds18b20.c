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
#include "owbus.h"
#include "ds18b20.h"

#define CRC8_DALLAS_INIT 0x00
#define DS18B20_CONVERT_T 0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

static uint8_t crc_8bit_dallas_init(uint8_t init, const void * data, int len)
{
    uint8_t i, tmp, dat, crc = init, *ptr = (uint8_t *)data;

    while (len--) {
        dat = *ptr++;
        for (i = 0; i < 8; i++) {
            tmp = (crc ^ dat) & 1;
            crc >>= 1;
            dat >>= 1;
            if (tmp)
                crc ^= 0x8c;
        }
    }
    return crc;
}

int ds18b20_convert_t(const void * rom)
{
    if (owbus_select(rom))
        return 1;

    owbus_write(DS18B20_CONVERT_T);

    return 0;
}

int ds18b20_read_t(const void * rom, uint16_t * data)
{
    int i;
    uint8_t scratchpad[9];
    uint8_t crc = CRC8_DALLAS_INIT;

    if (owbus_select(rom))
        return 1;

    owbus_write(DS18B20_READ_SCRATCHPAD);
    for (i = 0; i < sizeof(scratchpad); i++) {
        scratchpad[i] = owbus_read();
        crc = crc_8bit_dallas_init(crc, &scratchpad[i], 1);
    }

    if (data != NULL) {
        *data = scratchpad[0] | (scratchpad[1] << 8);
    }

    if (crc != 0)
        return 2;

    return 0;
}
