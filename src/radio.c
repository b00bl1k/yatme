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

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "nrf24l01p.h"
#include "radio.h"

#define RADIO_SPI_PORT GPIOA
#define RADIO_SPI_SCK_PIN GPIO5
#define RADIO_SPI_MOSI_PIN GPIO7
#define RADIO_SPI_MISO_PIN GPIO6
#define RADIO_CE_PORT GPIOA
#define RADIO_CE_PIN GPIO4
#define RADIO_CS_PORT GPIOA
#define RADIO_CS_PIN GPIO3
#define RADIO_PWR_PORT GPIOB
#define RADIO_PWR_PIN GPIO1

struct nrf24_device radio;

/* Assert chip select to low */
static void radio_cs_sel(bool sel)
{
    if (sel)
        gpio_clear(RADIO_CS_PORT, RADIO_CS_PIN);
    else
        gpio_set(RADIO_CS_PORT, RADIO_CS_PIN);
}

static void radio_ce_en(bool en)
{
    if (en)
        gpio_set(RADIO_CE_PORT, RADIO_CE_PIN);
    else
        gpio_clear(RADIO_CE_PORT, RADIO_CE_PIN);
}

static void radio_pwr_en(bool en)
{
    if (en)
        gpio_set(RADIO_PWR_PORT, RADIO_PWR_PIN);
    else
        gpio_clear(RADIO_PWR_PORT, RADIO_PWR_PIN);
}

static uint8_t radio_spi_xfer(uint8_t data)
{
    spi_send8(SPI1, data);
    return spi_read8(SPI1);
}

void radio_init()
{
    rcc_periph_clock_enable(RCC_SPI1);

    gpio_mode_setup(RADIO_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
        RADIO_SPI_SCK_PIN | RADIO_SPI_MOSI_PIN | RADIO_SPI_MISO_PIN);

    gpio_mode_setup(RADIO_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RADIO_CS_PIN);
    gpio_set_output_options(RADIO_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, RADIO_CS_PIN);

    gpio_mode_setup(RADIO_CE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RADIO_CE_PIN);
    gpio_set_output_options(RADIO_CE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, RADIO_CE_PIN);

    gpio_mode_setup(RADIO_PWR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RADIO_PWR_PIN);
    gpio_set_output_options(RADIO_PWR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, RADIO_PWR_PIN);

    spi_set_master_mode(SPI1);
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_8);
    spi_set_clock_polarity_0(SPI1);
    spi_set_clock_phase_0(SPI1);
    spi_set_full_duplex_mode(SPI1);
    spi_set_unidirectional_mode(SPI1);
    spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
    spi_fifo_reception_threshold_8bit(SPI1);
    spi_enable_software_slave_management(SPI1);
    spi_send_msb_first(SPI1);

    spi_set_nss_high(SPI1);
    spi_enable_ss_output(SPI1);

    radio_cs_sel(false);
    radio_ce_en(false);
    radio_pwr_en(false);

    spi_enable(SPI1);

    radio.pin_cs = radio_cs_sel;
    radio.pin_ce = radio_ce_en;
    radio.spi_xfer = radio_spi_xfer;
}

void radio_enable(bool en)
{
    radio_pwr_en(en);

    if (en) {
        board_delay_ms(100);
    }
}
