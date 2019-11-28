/****************************************************************************
 *   extdrv/cc1101.h
 *
 * CC1101 Low-Power Sub-1 GHz RF Transceiver.
 *
 * Copyright 2013 Nathael Pajani <nathael.pajani@ed3l.fr>
 */


/*
 * @author Athanassios Mavrogeorgiadis
 * @author TI CC1101 library developed by Athanassios Mavrogeorgiadis (tmav Electronics)
 *    as template based on TI C8051 SOURCE CODE swrc021f
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Datasheet:
 *
 * http://focus.ti.com/lit/ds/swrs061f/swrs061f.pdf
 */

#ifndef EXTDRV_CC1101_H
#define EXTDRV_CC1101_H

#include "lib/stdint.h"
#include "lib/stddef.h"

/******************************************************************************/
/* Control registers */

struct cc1101_configuration {
	uint8_t gdo_config[3];      /* 0x00 .. 0x02 - IOCFG2 .. IOCFG0 - GDOx output pin configuration */
	uint8_t fifo_thresholds;    /* 0x03 - FIFOTHR - RX FIFO and TX FIFO thresholds */
	uint8_t sync_word[2];       /* 0x04 .. 0x05 - SYNC1 .. SYNC0 - Sync word */
	uint8_t packet_length;      /* 0x06 - PKTLEN - Packet length */
	uint8_t pkt_ctrl[2];        /* 0x07 .. 0x08 - PKTCTRL1  .. PKTCTRL0 - Packet automation control */
	uint8_t device_addr;        /* 0x09 - ADDR - Device address */
	uint8_t channel_number;     /* 0x0A - CHANNR - Channel number */
	uint8_t freq_synth_ctrl[2];  /* 0x0B .. 0x0C - FSCTRL1 .. FSCTRL0 - Frequency synthesizer control */
	uint8_t freq_control[3];    /* 0x0D .. 0x0F - FREQ2 .. FREQ0 - Frequency control word*/
	uint8_t modem_config[5];    /* 0x10 .. 0x14 - MDMCFG4 .. MDMCFG0 - Modem configuration */
	uint8_t modem_deviation;    /* 0x15 - DEVIATN - Modem deviation setting */
	uint8_t radio_stm[3];       /* 0x16 .. 0x18 - MCSM2 .. MCSM0 - Main Radio Control State Machine configuration */
	uint8_t freq_offset_comp;   /* 0x19 - FOCCFG - Frequency Offset Compensation configuration */
	uint8_t bit_sync;           /* 0x1A - BSCFG - Bit Synchronization configuration */
	uint8_t agc_ctrl[3];        /* 0x1B .. 0x1D - AGCCTRL2 .. AGCCTRL0 - AGC control */
	uint8_t worevt_timeout[2];  /* 0x1E .. 0x1F - WOREVT1 .. WOREVT0 - Wake On Radio Event 0 timeout */
	uint8_t wake_on_radio;      /* 0x20 - WORCTRL - Wake On Radio control */
	uint8_t front_end_rx_cfg;   /* 0x21 - FREND1 - Front end RX configuration */
	uint8_t front_end_tx_cfg;   /* 0x22 - FREND0 - Front end TX configuration */
	uint8_t freq_synth_cal[4];  /* 0x23 .. 0x26 - FSCAL3 .. FSCAL0 - Frequency synthesizer calibration */
	uint8_t rc_osc_cal[2];      /* 0x27 .. 0x28 - RCCTRL1 .. RCCTRL0 - RC oscillator configuration */
	uint8_t freq_synth_cal_ctrl;  /* 0x29 - FSTEST - Frequency synthesizer calibration control */
	uint8_t production_test;    /* 0x2A - PTEST - Production test */
	uint8_t agc_test;           /* 0x2B - AGCTEST - AGC test */
	uint8_t test[3];            /* 0x2C .. 0x2E - TEST2 .. TEST0 - Various test settings */
	uint8_t unused_0;           /* 0x2F - empty */
	uint8_t unused_1[14];
	uint8_t pa_table;           /* 0x3E - PATABLE - */
	uint8_t fifo;               /* 0x3F - RX / TX FIFO - */
} __attribute__((packed));
#define CC1101_REGS(x)   (((uint8_t)offsetof(struct cc1101_configuration, x)) + (0x00))
#define CC1101_PATABLE        CC1101_REGS(pa_table)
#define CC1101_FIFO           CC1101_REGS(fifo)

#define CC1101_REGS_BURST(x)  (((uint8_t)offsetof(struct cc1101_configuration, x)) + (0x40))
#define CC1101_FIFO_BURST     CC1101_REGS_BURST(fifo)

#define CC1101_READ_OFFSET   0x80
#define CC1101_WRITE_OFFSET  0x00
#define CC1101_BURST_MODE    0x40

struct cc1101_status_regs {
	uint8_t part_number;   /* 0x30 - PARTNUM - Part number for CC1101 */
	uint8_t version;       /* 0x31 - VERSION - Current version number */
	uint8_t freq_test;     /* 0x32 - FREQEST - Frequency Offset Estimate */
	uint8_t link_quality;  /* 0x33 - LQI - Demodulator estimate for Link Quality */
	uint8_t sig_strength;  /* 0x34 - RSSI - Received signal strength indication */
	uint8_t state_machine; /* 0x35 - MARCSTATE - Control state machine state */
	uint8_t wor_timer[2];  /* 0x36 .. 0x37 - WORTIME1 .. WORTIME0 - WOR timer */
	uint8_t packet_status; /* 0x38 - PKTSTATUS - Current GDOx status and packet status */
	uint8_t pll_calibration;  /* 0x39 - VCO_VC_DAC - Current setting from PLL calibration module */
	uint8_t tx_bytes;      /* 0x3A - TXBYTES - Underflow and number of bytes in the TX FIFO */
	uint8_t rx_bytes;      /* 0x3B - RXBYTES - Overrflow and number of bytes in the TX FIFO */
	uint8_t rc_calib_status[2];  /* 0x3C .. 0x3D - RCCTRL1_STATUS .. RCCTRL0_STATUS - Last RC oscillator calibration result */
} __attribute__((packed));
#define CC1101_STATUS(x)   (((uint8_t)offsetof(struct cc1101_status_regs, x)) + (0x70))


struct cc1101_command_strobes {
	uint8_t reset;        /* 0x30 - SRES - Reset chip. */
	uint8_t start_freq_synth;  /* 0x31 - SFSTXON - Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in
	                              RX/TX: Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround). */
	uint8_t crystal_off;  /* 0x32 - SXOFF - Turn off crystal oscillator. */
	uint8_t synth_calibration;  /* 0x33 - SCAL - Calibrate frequency synthesizer and turn it off (enables quick start). */
	uint8_t state_rx;     /* 0x34 - SRX - Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1. */
	uint8_t state_tx;     /* 0x35 - STX - In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state
	                           and CCA is enabled: Only go to TX if channel is clear. */
	uint8_t state_idle;   /* 0x36 - SIDLE - Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable. */
	uint8_t freq_adj;     /* 0x37 - SAFC - Perform AFC adjustment of the frequency synthesizer */
	uint8_t state_wake_on_radio;  /* 0x38 - SWOR - Start automatic RX polling sequence (Wake-on-Radio) */
	uint8_t state_power_down;  /* 0x39 - SPWD - Enter power down mode when CSn goes high. */
	uint8_t flush_rx;     /* 0x3A - SFRX - Flush the RX FIFO buffer. */
	uint8_t flush_tx;     /* 0x3B - SFTX - Flush the TX FIFO buffer. */
	uint8_t wor_reset;    /* 0x3C - SWORRST - Reset real time clock. */
	uint8_t no_op;        /* 0x3D - SNOP - No operation. May be used to pad strobe commands to two bytes for simpler software. */
}  __attribute__((packed));
#define CC1101_CMD(x)   (((uint8_t)offsetof(struct cc1101_command_strobes, x)) + (0x30))


#define CC1101_RX_FIFO_OVERFLOW    (0x80)
#define CC1101_TX_FIFO_UNDERFLOW   (0x80)
#define CC1101_BYTES_IN_FIFO_MASK  (0x7F)
#define CC1101_FIFO_SIZE            64

#define CC1101_CRC_OK              0x80
#define CC1101_CARIER_SENSE        0x40
#define CC1101_CHANNEL_CLEAR       0x10


/* Errors definitions */
#define CC1101_ERR_BASE               10 /* Random number .... */
#define CC1101_ERR_RCV_PKT_TOO_BIG   (CC1101_ERR_BASE + 1)
#define CC1101_ERR_BUFF_TOO_SMALL    (CC1101_ERR_BASE + 2)
#define CC1101_ERR_INCOMPLET_PACKET  (CC1101_ERR_BASE + 3)
#define CC1101_ERR_OVERFLOW          (CC1101_ERR_BASE + 4)
#define CC1101_ERR_CRC               (CC1101_ERR_BASE + 5)


/* Definitions for chip status */
#define CC1101_RDY                        0x80
#define CC1101_STATE_MASK                 0x70
#define CC1101_STATE_IDLE                 0x00
#define CC1101_STATE_RX                   0x10
#define CC1101_STATE_TX                   0x20
#define CC1101_STATE_FSTON                0x30
#define CC1101_STATE_CALIBRATE            0x40
#define CC1101_STATE_SETTLING             0x50
#define CC1101_STATE_RXFIFO_OVERFLOW      0x60
#define CC1101_STATE_TXFIFO_UNDERFLOW     0x70
#define CC1101_STATE_FIFO_BYTES_MASK      0x0F



/***************************************************************************** */
/* SPI registers and commands access Wrappers */

/* Reset CC1101 chip */
void cc1101_reset(void);
/* Power Up Reset is the same as a usual reset, the function will only wait longer for
 *   the "chip ready" signal before starting SPI transfers .... */
void cc1101_power_up_reset(void);

void cc1101_flush_tx_fifo(void);
void cc1101_flush_rx_fifo(void);

/***************************************************************************** */
/* Read global status byte */
uint8_t cc1101_read_status(void);
/* Read packet status byte */
uint8_t cc1101_read_pkt_status(void);

/***************************************************************************** */
/* Enter Rx mode */
void cc1101_enter_rx_mode(void);
/* Prepare for entering Tx mode by moving to FSTXON state */
void cc1101_enter_fstxon_state(void);


/***************************************************************************** */
/* Signal strength and link quality */

/* Return the signal strength indication based in the last packet received */
uint8_t cc1101_get_signal_strength_indication(void);
/* Return the link quality indication based in the last packet received */
uint8_t cc1101_get_link_quality(void);

/* Request a calibration */
void cc1101_send_calibration_request(void);

/***************************************************************************** */
/* Rx fifo state :
 * Return 0 when fifo is empty, or number of remaining bytes when non empty and no
 *    overflow occured.
 * Return -1 when an overflow occured.
 * Upon overflow, the radio is placed in idle state and the RX fifo flushed.
 * Else the radio is placed in RX state.
 */
int cc1101_rx_fifo_state(void);

/* Tx fifo state :
 * Return 0 when fifo is empty, or number of remaining bytes when non empty and no
 *    underflow occured.
 * Return a negative value on error:
 *     when an underflow occured, return value is -1
 *     on other errors return value is (-1) * (global status byte)
 * Upon error, the radio is placed in idle state and the TX fifo flushed.
 * Else the radio is placed in TX state.
 */
int cc1101_tx_fifo_state(void);


/***************************************************************************** */
/* Send packet
 * When using a packet oriented communication with packet size and address included
 *   in the packet, these must be included in the packet by the software before
 *   calling this function.
 * In order to send packets of more than 64 bytes (including length and address) the
 *   user will have to write his own function.
 */
int cc1101_send_packet(uint8_t* buffer, uint8_t size);

/* Receive packet
 * This function can be used to receive a packet of variable packet length (first byte
 *   in the packet must be the length byte). The packet length should not exceed
 *   the RX FIFO size.
 * The buffer provided must be able to hold "size" bytes from the receive fifo.
 * Returns the size filled in the buffer if everything went right.
 * Returns a negative value upon error.
 *    CC1101_ERR_RCV_PKT_TOO_BIG: Received packet said to be bigger than fifo size
 *    CC1101_ERR_BUFF_TOO_SMALL: Provided buffer is smaller than received packet
 *    CC1101_ERR_INCOMPLET_PACKET: Fifo hodls less data than received packet length
 *    CC1101_OVERFLOW: RX fifo overflow
 *    CC1101_ERR_CRC: Packet CRC error. The fifo has been flushed.
 *    In every case we read as many data as possible, which is the provided buffer
 *    size in case of CC1101_ERR_BUFF_TOO_SMALL. In other cases, it is the maximum
 *    possible depending on available bytes in fifo (status provides this), the
 *    buffer size, and the packet size (provided in buffer[0]).
 */
int cc1101_receive_packet(uint8_t* buffer, uint8_t size, uint8_t* status);



/***************************************************************************** */
/* CC1101 Initialisation */

/* Change the CC1101 address
 * Only packets addressed to the specified address (or broadcasted) will be
*    received.
 * Adresses 0x00 and 0xFF are broadcast
 * This function places the CC1101 chip in idle state.
 */
void cc1101_set_address(uint8_t address);

/* Set current channel to use.
 * The caller is responsible for checking that the channel spacing and channel bandwith are configures
 * correctly to prevent overlaping channels, or to use only non-overlaping channel numbers.
 * This function places the CC1101 chip in idle state.
 */
void cc1101_set_channel(uint8_t chan);

/* Enter power down mode
 * Power down mode is exited by setting the chip select pin low (any access to the CC1101 will do so)
 */
void cc1101_power_down(void);

/* Change a configuration byte.
 * This function places the CC1101 chip in idle state.
 */
void cc1101_set_config(uint8_t byte_addr, uint8_t value);

/* Configure pins, reset the CC1101, and put the CC1101 chip in idle state */
void cc1101_init(uint8_t ssp_num, const struct pio* cs_pin, const struct pio* miso_pin);

/* Write / send all the configuration register values to the CC1101 chip
 * This function places the CC1101 chip in idle state.
 */
void cc1101_config(void);

/* Update CC1101 config
 * Arguments are the settings table which is a table of address and value pairs,
 *   and the table length, which must be even.
 * This function places the CC1101 chip in idle state.
 */
void cc1101_update_config(uint8_t* settings, uint8_t len);

/* Change PA Table value */
void cc1101_set_patable(uint8_t val);

#endif /* EXTDRV_CC1101_H */
