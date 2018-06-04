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

#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__

#include <stdbool.h>
#include <stdint.h>

/* Commands */
#define NRF24_CMD_R_REGISTER 0x00
#define NRF24_CMD_W_REGISTER 0x20
#define NRF24_CMD_R_RX_PAYLOAD 0x61
#define NRF24_CMD_W_TX_PAYLOAD 0xA0
#define NRF24_CMD_FLUSH_TX 0xE1
#define NRF24_CMD_FLUSH_RX 0xE2
#define NRF24_CMD_REUSE_TX_PL 0xE3
#define NRF24_CMD_R_RX_PL_WID 0x60
#define NRF24_CMD_W_ACK_PAYLOAD 0xA8
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0
#define NRF24_CMD_NOP 0xFF

/* Registers */
#define NRF24_CONFIG 0x00
#define NRF24_EN_AA 0x01
#define NRF24_EN_RXADDR 0x02
#define NRF24_SETUP_AW 0x03
#define NRF24_SETUP_RETR 0x04
#define NRF24_RF_CH 0x05
#define NRF24_RF_SETUP 0x06
#define NRF24_STATUS 0x07
#define NRF24_OBSERVE_TX 0x08
#define NRF24_RPD 0x09
#define NRF24_RX_ADDR_P0 0x0A
#define NRF24_RX_ADDR_P1 0x0B
#define NRF24_RX_ADDR_P2 0x0C
#define NRF24_RX_ADDR_P3 0x0D
#define NRF24_RX_ADDR_P4 0x0E
#define NRF24_RX_ADDR_P5 0x0F
#define NRF24_TX_ADDR 0x10
#define NRF24_RX_PW_P0 0x11
#define NRF24_RX_PW_P1 0x12
#define NRF24_RX_PW_P2 0x13
#define NRF24_RX_PW_P3 0x14
#define NRF24_RX_PW_P4 0x15
#define NRF24_RX_PW_P5 0x16
#define NRF24_FIFO_STATUS 0x17
#define NRF24_DYNPD 0x1C
#define NRF24_FEATURE 0x1D

/* CONFIG Register */
#define NRF24_CONFIG_PRIM_RX (1 << 0)
#define NRF24_CONFIG_PWR_UP (1 << 1)
#define NRF24_CONFIG_CRCO (1 << 2)
#define NRF24_CONFIG_EN_CRC (1 << 3)
#define NRF24_CONFIG_MASK_MAX_RT (1 << 4)
#define NRF24_CONFIG_MASK_TX_DS (1 << 5)
#define NRF24_CONFIG_MASK_RX_DR (1 << 6)

/* EN_AA Register */
#define NRF24_EN_AA_P0 (1 << 0)
#define NRF24_EN_AA_P1 (1 << 1)
#define NRF24_EN_AA_P2 (1 << 2)
#define NRF24_EN_AA_P3 (1 << 3)
#define NRF24_EN_AA_P4 (1 << 4)
#define NRF24_EN_AA_P5 (1 << 5)

/* EN_RXADDR Register */
#define NRF24_EN_RXADDR_ERX_P0 (1 << 0)
#define NRF24_EN_RXADDR_ERX_P1 (1 << 1)
#define NRF24_EN_RXADDR_ERX_P2 (1 << 2)
#define NRF24_EN_RXADDR_ERX_P3 (1 << 3)
#define NRF24_EN_RXADDR_ERX_P4 (1 << 4)
#define NRF24_EN_RXADDR_ERX_P5 (1 << 5)

/* SETUP_AW Register */
#define NRF24_SETUP_AW_AW_SHIFT 0
#define NRF24_SETUP_AW_AW_MASK (0x3 << NRF24_SETUP_AW_AW_SHIFT)
#define NRF24_SETUP_AW_AW_3 (0x1 << NRF24_SETUP_AW_AW_SHIFT)
#define NRF24_SETUP_AW_AW_4 (0x2 << NRF24_SETUP_AW_AW_SHIFT)
#define NRF24_SETUP_AW_AW_5 (0x3 << NRF24_SETUP_AW_AW_SHIFT)

/* SETUP_RETR Register */
#define NRF24_SETUP_RETR_ARD_SHIFT 4
#define NRF24_SETUP_RETR_ARD_MASK (0xF << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_250US (0x0 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_500US (0x1 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_750US (0x2 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_1000US (0x3 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_1250US (0x4 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_1500US (0x5 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_1750US (0x6 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_2000US (0x7 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_2250US (0x8 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_2500US (0x9 << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_2750US (0xA << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_3000US (0xB << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_3250US (0xC << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_3500US (0xD << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_3750US (0xE << NRF24_SETUP_RETR_ARD_SHIFT)
#define NRF24_SETUP_RETR_ARD_4000US (0xF << NRF24_SETUP_RETR_ARD_SHIFT)

#define NRF24_SETUP_RETR_ARC_SHIFT 0
#define NRF24_SETUP_RETR_ARC_MASK (0xF << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_DISABLED (0x0 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_1 (0x1 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_2 (0x2 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_3 (0x3 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_4 (0x4 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_5 (0x5 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_6 (0x6 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_7 (0x7 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_8 (0x8 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_9 (0x9 << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_10 (0xA << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_11 (0xB << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_12 (0xC << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_13 (0xD << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_14 (0xE << NRF24_SETUP_RETR_ARC_SHIFT)
#define NRF24_SETUP_RETR_ARC_15 (0xF << NRF24_SETUP_RETR_ARC_SHIFT)

/* RF_CH Register */
#define NRF24_RF_CH_SHIFT 0
#define NRF24_RF_CH_MASK (0x7F << NRF24_RF_CH_SHIFT)

/* RF_SETUP Register */
#define NRF24_RF_SETUP_RF_PWR_SHIFT 1
#define NRF24_RF_SETUP_RF_PWR_MASK (0x3 << NRF24_RF_SETUP_RF_PWR_SHIFT)
#define NRF24_RF_SETUP_RF_PWR_M18DBM (0x0 << NRF24_RF_SETUP_RF_PWR_SHIFT)
#define NRF24_RF_SETUP_RF_PWR_M12DBM (0x1 << NRF24_RF_SETUP_RF_PWR_SHIFT)
#define NRF24_RF_SETUP_RF_PWR_M6DBM (0x2 << NRF24_RF_SETUP_RF_PWR_SHIFT)
#define NRF24_RF_SETUP_RF_PWR_0DBM (0x3 << NRF24_RF_SETUP_RF_PWR_SHIFT)

#define NRF24_RF_SETUP_RF_DR_HIGH (1 << 3)
#define NRF24_RF_SETUP_PLL_LOCK (1 << 4)
#define NRF24_RF_SETUP_RF_DR_LOW (1 << 5)
#define NRF24_RF_SETUP_CONT_WAVE (1 << 7)

/* STATUS Register */
#define NRF24_STATUS_TX_FULL (1 << 0)

#define NRF24_STATUS_RX_P_NO_SHIFT 1
#define NRF24_STATUS_RX_P_NO_MASK (0x7 << NRF24_STATUS_RX_P_NO_SHIFT)
#define NRF24_STATUS_RX_P_0 (0x0 << NRF24_STATUS_RX_P_NO_SHIFT)
#define NRF24_STATUS_RX_P_1 (0x1 << NRF24_STATUS_RX_P_NO_SHIFT)
#define NRF24_STATUS_RX_P_2 (0x2 << NRF24_STATUS_RX_P_NO_SHIFT)
#define NRF24_STATUS_RX_P_3 (0x3 << NRF24_STATUS_RX_P_NO_SHIFT)
#define NRF24_STATUS_RX_P_4 (0x4 << NRF24_STATUS_RX_P_NO_SHIFT)
#define NRF24_STATUS_RX_P_5 (0x5 << NRF24_STATUS_RX_P_NO_SHIFT)
#define NRF24_STATUS_RX_P_NO_EMPTY (0x7 << NRF24_STATUS_RX_P_NO_SHIFT)

#define NRF24_STATUS_MAX_RT (1 << 4)
#define NRF24_STATUS_TX_DS (1 << 5)
#define NRF24_STATUS_RX_DR (1 << 6)

/* OBSERVE_TX Register */
#define NRF24_OBSERVE_TX_ARC_CNT_SHIFT 0
#define NRF24_OBSERVE_TX_ARC_CNT_MASK (0xF << NRF24_OBSERVE_TX_ARC_CNT_SHIFT)

#define NRF24_OBSERVE_TX_PLOS_CNT_SHIFT 4
#define NRF24_OBSERVE_TX_PLOS_CNT_MASK (0xF << NRF24_OBSERVE_TX_PLOS_CNT_SHIFT)

/* RPD Register */
#define NRF24_RPD_RPD (1 << 0)

/* RX_PW_P(0-5) Register */
#define NRF24_RX_PW_PN_SHIFT 0
#define NRF24_RX_PW_PN_MASK (0x3F << NRF24_RX_PW_PN_SHIFT)

/* FIFO_STATUS Register */
#define NRF24_FIFO_STATUS_RX_EMPTY (1 << 0)
#define NRF24_FIFO_STATUS_RX_FULL (1 << 1)
#define NRF24_FIFO_STATUS_TX_EMPTY (1 << 4)
#define NRF24_FIFO_STATUS_TX_FULL (1 << 5)
#define NRF24_FIFO_STATUS_TX_REUSE (1 << 6)

/* DYNPD Register */
#define NRF24_DYNPD_DPL_P0 (1 << 0)
#define NRF24_DYNPD_DPL_P1 (1 << 1)
#define NRF24_DYNPD_DPL_P2 (1 << 2)
#define NRF24_DYNPD_DPL_P3 (1 << 3)
#define NRF24_DYNPD_DPL_P4 (1 << 4)
#define NRF24_DYNPD_DPL_P5 (1 << 5)

/* FEATURE Register */
#define NRF24_FEATURE_EN_DYN_ACK (1 << 0)
#define NRF24_FEATURE_EN_ACK_PAY (1 << 1)
#define NRF24_FEATURE_EN_DPL (1 << 2)

#define NRF24_RX_PIPES_MAX 6
#define NRF24_ADDR_SIZE_MAX 5

typedef void (* nrf24_pin_t)(bool);
typedef uint8_t (* nrf24_spi_xfer_t)(uint8_t);
typedef void (* nrf24_delay_us_t)(uint32_t);

enum nrf24_crc {
    NRF24_CRC_DISABLE,
    NRF24_CRC_8,
    NRF24_CRC_16
};

enum nrf24_aw {
    NRF24_AW_3 = NRF24_SETUP_AW_AW_3,
    NRF24_AW_4 = NRF24_SETUP_AW_AW_4,
    NRF24_AW_5 = NRF24_SETUP_AW_AW_5
};

enum nrf24_power {
    NRF24_PWR_0 = NRF24_RF_SETUP_RF_PWR_0DBM,
    NRF24_PWR_M6 = NRF24_RF_SETUP_RF_PWR_M6DBM,
    NRF24_PWR_M12 = NRF24_RF_SETUP_RF_PWR_M12DBM,
    NRF24_PWR_M18 = NRF24_RF_SETUP_RF_PWR_M18DBM
};

enum nrf24_datarate {
    NRF24_DR_250_KBPS = NRF24_RF_SETUP_RF_DR_LOW,
    NRF24_DR_1_MBPS = 0,
    NRF24_DR_2_MBPS = NRF24_RF_SETUP_RF_DR_HIGH
};

enum nrf24_ard {
    NRF24_ARD_250US = NRF24_SETUP_RETR_ARD_250US,
    NRF24_ARD_500US = NRF24_SETUP_RETR_ARD_500US,
    NRF24_ARD_750US = NRF24_SETUP_RETR_ARD_750US,
    NRF24_ARD_1000US = NRF24_SETUP_RETR_ARD_1000US,
    NRF24_ARD_1250US = NRF24_SETUP_RETR_ARD_1250US,
    NRF24_ARD_1500US = NRF24_SETUP_RETR_ARD_1500US,
    NRF24_ARD_1750US = NRF24_SETUP_RETR_ARD_1750US,
    NRF24_ARD_2000US = NRF24_SETUP_RETR_ARD_2000US,
    NRF24_ARD_2250US = NRF24_SETUP_RETR_ARD_2250US,
    NRF24_ARD_2500US = NRF24_SETUP_RETR_ARD_2500US,
    NRF24_ARD_2750US = NRF24_SETUP_RETR_ARD_2750US,
    NRF24_ARD_3000US = NRF24_SETUP_RETR_ARD_3000US,
    NRF24_ARD_3250US = NRF24_SETUP_RETR_ARD_3250US,
    NRF24_ARD_3500US = NRF24_SETUP_RETR_ARD_3500US,
    NRF24_ARD_3750US = NRF24_SETUP_RETR_ARD_3750US,
    NRF24_ARD_4000US = NRF24_SETUP_RETR_ARD_4000US
};

struct nrf24_device {
    nrf24_pin_t pin_cs;
    nrf24_pin_t pin_ce;
    nrf24_spi_xfer_t spi_xfer;
    nrf24_delay_us_t delay_us;

    uint8_t reg_dynpd;
    uint8_t reg_en_aa;
    uint8_t reg_en_rxaddr;
};

struct nrf24_init_def {
    enum nrf24_crc crc;
    enum nrf24_power rf_pwr;
    enum nrf24_datarate rf_dr;
    uint8_t rf_ch; /*!< RF channel: 0 .. 127 */
    enum nrf24_ard ard;
    uint8_t arc; /*!< Auto Retransmit Count: 0 .. 15 */
    bool dpl;
    bool ack_pay;
    bool dyn_ack;
};

struct nrf24_rx_pipe {
    bool enabled;
    bool dpl; /* Dynamic payload width */
    bool aack; /* Auto ack */
    union {
        uint8_t rx01[NRF24_ADDR_SIZE_MAX];
        uint8_t rx2345;
    } addr;
    uint8_t pw; /* Payload width */
};

struct nrf24_pipes_def {
    enum nrf24_aw aw;
    uint8_t tx_addr[NRF24_ADDR_SIZE_MAX];
    struct nrf24_rx_pipe rx_pipes[NRF24_RX_PIPES_MAX];
};

uint8_t nrf24_cmd(struct nrf24_device *, uint8_t);
uint8_t nrf24_read_reg(struct nrf24_device *, uint8_t);
uint8_t nrf24_write_reg(struct nrf24_device *, uint8_t, uint8_t);
uint8_t nrf24_read_array(struct nrf24_device *, uint8_t, void *, int);
uint8_t nrf24_write_array(struct nrf24_device *, uint8_t, const void *, int);
uint8_t nrf24_read_reg_array(struct nrf24_device *, uint8_t, void * dst, int);
uint8_t nrf24_write_reg_array(struct nrf24_device *, uint8_t, const void *, int);
bool nrf24_init(struct nrf24_device *, const struct nrf24_init_def *);
void nrf24_setup_pipes(struct nrf24_device *, const struct nrf24_pipes_def *);

#endif /* ~__NRF24L01P_H__ */
