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

static void setup_rx_pipe(struct nrf24_device * dev,
    const struct nrf24_rx_pipe * pipe, int pipe_no, int aw)
{
    uint8_t pipe_mask = 1 << pipe_no;

    if (pipe->enabled == false)
        return;

    dev->reg_en_rxaddr |= pipe_mask;

    if (pipe->dpl)
        dev->reg_dynpd |= pipe_mask;
    else
        nrf24_write_reg(dev, NRF24_RX_PW_P0 + pipe_no, pipe->pw);

    if (pipe->aack)
        dev->reg_en_aa |= pipe_mask;

    if (pipe_no < 2) {
        nrf24_write_reg_array(dev, NRF24_RX_ADDR_P0 + pipe_no,
            pipe->addr.rx01, aw);
    }
    else {
        nrf24_write_reg_array(dev, NRF24_RX_ADDR_P0 + pipe_no,
            &pipe->addr.rx2345, 1);
    }
}

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

uint8_t nrf24_read_array(struct nrf24_device * dev, uint8_t cmd, void * dst,
    int size)
{
    uint8_t status;
    uint8_t * ptr = dst;

    dev->pin_cs(true);
    status = dev->spi_xfer(cmd);
    for (; size > 0; size--, ptr++) {
        *ptr = dev->spi_xfer(NRF24_CMD_NOP);
    }
    dev->pin_cs(false);

    return status;
}

uint8_t nrf24_write_array(struct nrf24_device * dev, uint8_t cmd,
    const void * src, int size)
{
    uint8_t status;
    const uint8_t * ptr = src;

    dev->pin_cs(true);
    status = dev->spi_xfer(cmd);
    for (; size > 0; size--, ptr++) {
        dev->spi_xfer(*ptr);
    }
    dev->pin_cs(false);

    return status;
}

uint8_t nrf24_read_reg_array(struct nrf24_device * dev, uint8_t reg, void * dst,
    int size)
{
    return nrf24_read_array(dev, NRF24_CMD_R_REGISTER | reg, dst, size);
}

uint8_t nrf24_write_reg_array(struct nrf24_device * dev, uint8_t reg,
    const void * src, int size)
{
    return nrf24_write_array(dev, NRF24_CMD_W_REGISTER | reg, src, size);
}

bool nrf24_init(struct nrf24_device * dev, const struct nrf24_init_def * init_def)
{
    uint8_t conf_reg;
    uint8_t tmp_reg;

    /* Setup config (and power down) */
    conf_reg = 0x0;
    if (init_def->crc != NRF24_CRC_DISABLE) {
        conf_reg = NRF24_CONFIG_EN_CRC;
        if (init_def->crc == NRF24_CRC_16)
            conf_reg |= NRF24_CONFIG_CRCO;
    }
    nrf24_write_reg(dev, NRF24_CONFIG, conf_reg);

    if (nrf24_read_reg(dev, NRF24_CONFIG) != conf_reg)
        return false;

    nrf24_write_reg(dev, NRF24_EN_AA, 0x0);
    nrf24_write_reg(dev, NRF24_EN_RXADDR, 0x0);
    nrf24_write_reg(dev, NRF24_DYNPD, 0x0);

    /* RF channel */
    nrf24_write_reg(dev, NRF24_RF_CH,
        (init_def->rf_ch << NRF24_RF_CH_SHIFT) & NRF24_RF_CH_MASK);

    /* Ouput power and datarate */
    nrf24_write_reg(dev, NRF24_RF_SETUP, init_def->rf_pwr | init_def->rf_dr);

    /* Auto retransmit delay and count */
    nrf24_write_reg(dev, NRF24_SETUP_RETR,
        ((init_def->arc << NRF24_SETUP_RETR_ARC_SHIFT) & NRF24_SETUP_RETR_ARC_MASK) |
        init_def->ard);

    /* Enable features */
    tmp_reg = 0x0;
    if (init_def->dpl)
        tmp_reg |= NRF24_FEATURE_EN_DPL;
    if (init_def->ack_pay)
        tmp_reg |= NRF24_FEATURE_EN_ACK_PAY;
    if (init_def->dyn_ack)
        tmp_reg |= NRF24_FEATURE_EN_DYN_ACK;
    nrf24_write_reg(dev, NRF24_FEATURE, tmp_reg);

    /* Power up */
    conf_reg |= NRF24_CONFIG_PWR_UP;
    nrf24_write_reg(dev, NRF24_CONFIG, conf_reg);

    return true;
}

void nrf24_setup_pipes(struct nrf24_device * dev,
    const struct nrf24_pipes_def * pipes)
{
    int aw;
    int pipe_no;

    nrf24_write_reg(dev, NRF24_SETUP_AW, pipes->aw);
    if (pipes->aw == NRF24_AW_3)
        aw = 3;
    else if (pipes->aw == NRF24_AW_4)
        aw = 4;
    else
        aw = 5;

    nrf24_write_reg_array(dev, NRF24_TX_ADDR, pipes->tx_addr, aw);

    dev->reg_dynpd = 0x0;
    dev->reg_en_aa = 0x0;
    dev->reg_en_rxaddr = 0x0;

    for (pipe_no = 0; pipe_no < NRF24_RX_PIPES_MAX; pipe_no++) {
        setup_rx_pipe(dev, &pipes->rx_pipes[pipe_no], pipe_no, aw);
    }

    nrf24_write_reg(dev, NRF24_DYNPD, dev->reg_dynpd);
    nrf24_write_reg(dev, NRF24_EN_AA, dev->reg_en_aa);
    nrf24_write_reg(dev, NRF24_EN_RXADDR, dev->reg_en_rxaddr);
}
