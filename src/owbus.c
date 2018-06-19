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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include "owbus.h"

static void owbus_rx_flush()
{
    while (usart_get_flag(USART1, USART_ISR_RXNE))
        usart_recv(USART1);
}

static uint8_t owbus_xfer(uint8_t data)
{
    usart_send(USART1, data);

    for (volatile uint32_t t = 0; t < 0x100000; t++) {
        if (usart_get_flag(USART1, USART_ISR_RXNE))
            return usart_recv(USART1);
    }

    return 0xf0;
}

void owbus_init()
{
    rcc_periph_clock_enable(RCC_USART1);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

    gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    usart_enable(USART1);
}

int owbus_reset()
{
    uint8_t byte;

    if (gpio_get(GPIOA, GPIO10) == 0)
        return -1;

    owbus_rx_flush();

    usart_set_baudrate(USART1, 9600);
    byte = owbus_xfer(0xf0);
    usart_set_baudrate(USART1, 115200);

    if (byte == 0xf0)
        return -2;
    else
        return 0;
}

void owbus_write(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        if (data & 1)
            owbus_xfer(0xff);
        else
            owbus_xfer(0x00);
        data >>= 1;
    }
}

uint8_t owbus_read()
{
    uint8_t res = 0;

    for (int i = 0; i < 8; i++) {
        res = res >> 1;
        if (owbus_xfer(0xff) == 0xff)
            res |= 0x80;
    }

    return res;
}

int owbus_read_rom(void * dst)
{
    int i, res = 0;
    uint8_t * data = (uint8_t *)dst;

    res = owbus_reset();

    if (res == 0) {
        owbus_write(OWBUS_READ_ROM);
        for (i = 0; i < OWBUS_ROM_LEN; i++)
            data[i] = owbus_read();
    }

    return res;
}

int owbus_select(const void * rom)
{
    int i, res = 0;

    res = owbus_reset();

    if (res == 0) {
        if (rom == 0) {
            owbus_write(OWBUS_SKIP_ROM);
        }
        else {
            owbus_write(OWBUS_MATCH_ROM);
            for (i = 0; i < OWBUS_ROM_LEN; i++)
                owbus_write(((const uint8_t *)rom)[i]);
        }
    }

    return res;
}
