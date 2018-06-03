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

#include "nrf24l01p.h"

/**
 * @brief NRF24 write command
 *
 * @param[in] dev pointer to device descriptor
 * @param[in] cmd command code
 *
 * @return status of the device
 */
uint8_t nrf24_cmd(struct nrf24_device * dev, uint8_t cmd)
{
    uint8_t status;

    dev->pin_cs(true);
    status = dev->spi_xfer(cmd);
    dev->pin_cs(false);

    return status;
}

/**
 * @brief NRF24 read register
 *
 * @param[in] dev pointer to device descriptor
 * @param[in] reg register code
 *
 * @return value of the register
 */
uint8_t nrf24_read_reg(struct nrf24_device * dev, uint8_t reg)
{
    uint8_t result;

    dev->pin_cs(true);
    dev->spi_xfer(NRF24_CMD_R_REGISTER | reg);
    result = dev->spi_xfer(NRF24_CMD_NOP);
    dev->pin_cs(false);

    return result;
}

/**
 * @brief NRF24 write to register
 *
 * @param[in] dev pointer to device descriptor
 * @param[in] reg register code
 * @param[in] val new register value
 *
 * @return status of the device
 */
uint8_t nrf24_write_reg(struct nrf24_device * dev, uint8_t reg, uint8_t val)
{
    uint8_t status;

    dev->pin_cs(true);
    status = dev->spi_xfer(NRF24_CMD_W_REGISTER | reg);
    dev->spi_xfer(val);
    dev->pin_cs(false);

    return status;
}
