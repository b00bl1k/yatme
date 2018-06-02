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

#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include "board.h"

static volatile uint32_t ticks;

static void clock_init(void)
{
    rcc_osc_on(RCC_HSI);
    rcc_wait_for_osc_ready(RCC_HSI);
    rcc_set_sysclk_source(RCC_HSI);

    rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
    rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

    flash_set_ws(FLASH_ACR_LATENCY_000_024MHZ);

    rcc_apb1_frequency = 8000000;
    rcc_ahb_frequency = 8000000;

    systick_set_reload(rcc_ahb_frequency / 1000UL);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();

    rcc_periph_clock_enable(RCC_PWR);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
}

static void enter_standby_mode(void)
{
    pwr_clear_wakeup_flag();
    pwr_set_standby_mode();
    SCB_SCR |= SCB_SCR_SLEEPDEEP;
    __WFI();
}

void board_init()
{
    clock_init();
}

void board_sleep()
{
    enter_standby_mode();
}

void board_delay_ms(uint32_t ms)
{
    volatile uint32_t last = ticks;

    while (ticks - last < ms) {
    }
}

void sys_tick_handler(void)
{
    ticks++;
}
