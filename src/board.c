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
#include <libopencm3/stm32/rtc.h>
#include "board.h"
#include "owbus.h"

/* RTC alarm X sub second register */
#define RTC_ALRMXSRR_MASKSS_SHIFT (24)
#define RTC_ALRMXSSR_MASKSS_MASK (0xf)
#define RTC_ALRMXSSR_MASKSS_NO (0)
#define RTC_ALRMXSSR_MASKSS_ALL (0xf)
#define RTC_ALRMXSSR_SS_MASK (0x7fff)

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

    rcc_periph_clock_enable(RCC_PWR);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    systick_set_reload(STK_RVR_RELOAD);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
}

static uint8_t rtc_add_minutes(uint8_t shift)
{
    uint32_t time = RTC_TR;
    uint8_t min = (time >> 8) & 0xff;

    /* From BCD */
    min = (min & 0xf) + ((min >> 4) * 10);

    min = (min + shift) % 60;

    /* To BCD */
    min = (min % 10) | ((min / 10) << 4);

    return min;
}

static void rtc_set_alarma(uint32_t mask, uint8_t date, uint8_t hours,
    uint8_t minutes, uint8_t seconds, uint8_t ss_mask, uint16_t ss_value)
{
    rtc_unlock();
    RTC_CR &= ~RTC_CR_ALRAE;

    while(!(RTC_ISR & RTC_ISR_ALRAWF)) {
    }

    RTC_ALRMAR = mask | (date << RTC_ALRMXR_DU_SHIFT) |
        (hours << RTC_ALRMXR_HU_SHIFT) | (minutes << RTC_ALRMXR_MNU_SHIFT) |
        (seconds << RTC_ALRMXR_SU_SHIFT);
    RTC_ALRMASSR = (ss_mask << RTC_ALRMXSRR_MASKSS_SHIFT) | ss_value;

    RTC_CR |= RTC_CR_ALRAE;
    rtc_lock();
}

static void rtc_init(void)
{
   pwr_disable_backup_domain_write_protect();

    if (pwr_get_standby_flag()) {
        pwr_clear_standby_flag();
    }
    else {
        RCC_BDCR &= ~RCC_BDCR_RTCEN;

        rcc_periph_reset_pulse(RST_BACKUPDOMAIN);
        rcc_osc_on(RCC_LSI);
        rcc_wait_for_osc_ready(RCC_LSI);
        RCC_BDCR = (RCC_BDCR & ~RCC_BDCR_RTCSEL) |
                        RCC_BDCR_RTCSEL_LSI;

        rcc_enable_rtc_clock();
        rtc_unlock();

        RTC_ISR |= RTC_ISR_INIT;
        while ((RTC_ISR & RTC_ISR_INITF) == 0) {
        }

        rtc_set_prescaler(311, 127);

        RTC_ISR &= ~(RTC_ISR_INIT);
        rtc_lock();

        RCC_BDCR |= RCC_BDCR_RTCEN;
    }

    rtc_set_alarma(RTC_ALRMXR_MSK4 | RTC_ALRMXR_MSK3,
        0x00, 0x00, rtc_add_minutes(2), 0x00, 0, 0);

    rtc_unlock();
    RTC_CR |= RTC_CR_ALRAIE;
    rtc_lock();

    nvic_enable_irq(NVIC_RTC_IRQ);
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
    owbus_init();
    rtc_init();
}

void board_sleep()
{
    enter_standby_mode();
}

void board_delay_us(uint32_t us)
{
    uint32_t goal = STK_RVR_RELOAD - (us * 8);

    systick_clear();
    while (systick_get_value() > goal) {
    }
}
