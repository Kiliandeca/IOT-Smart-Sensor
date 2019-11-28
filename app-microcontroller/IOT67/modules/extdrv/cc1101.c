/****************************************************************************
 *  extdrv/cc1101.c
 *
 * Copyright 2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *************************************************************************** */

#include "lib/errno.h"
#include "core/system.h"
#include "core/pio.h"
#include "lib/string.h"
#include "drivers/ssp.h"
#include "drivers/gpio.h"
#include "extdrv/cc1101.h"

/* Driver for the CC1101 Sub-1GHz RF transceiver from Texas Instrument.
 * Refer to CC1101 documentation for more information (swrs061i.pdf)
 */


/***************************************************************************** */
/*                CC1101                                                       */
/***************************************************************************** */
struct cc1101_device {
	uint8_t spi_num;
	struct pio cs_pin; /* SPI CS pin, used as chip select */
	struct pio miso_pin; /* SPI MISO pin, used to monitor "device ready" from CC1101 chip */
	/* Signal indication */
	uint8_t rx_sig_strength; /* Received signal strength indication */
	uint8_t link_quality; /* link quality */
};
static struct cc1101_device cc1101 = {
	.rx_sig_strength = 0,
	.link_quality = 0,
};


/***************************************************************************** */
/* Main SPI transfer function */
uint8_t cc1101_spi_transfer(uint8_t addr, uint8_t* out, uint8_t* in, uint8_t size)
{
	struct lpc_gpio* gpio = LPC_GPIO_REGS(cc1101.cs_pin.port);
	uint8_t status = 0;

	/* Set CS Low */
	gpio->clear = (1 << cc1101.cs_pin.pin);

	/* Wait for ready state (GDO_1 / MISO going low) */
	do {} while (gpio->in & (0x01 << cc1101.miso_pin.pin));

	/* Send address and get global status */
	status = (uint8_t)spi_transfer_single_frame(cc1101.spi_num, (uint16_t)addr);
	/* Perform transfer */
	if (size != 0) {
		spi_transfer_multiple_frames(cc1101.spi_num, out, in, size, 8);
	}
	/* Release Chip select */
	gpio->set = (1 << cc1101.cs_pin.pin);

	return status;
}


/* Read and return single register value */
static uint8_t cc1101_read_reg(uint8_t addr)
{
	uint8_t ret = 0;
	cc1101_spi_transfer((addr | CC1101_READ_OFFSET), NULL, &ret, 1);
	return ret;
}
/* Read nb registers from start_addr into buffer. Return the global status byte. */
static uint8_t cc1101_read_burst_reg(uint8_t start_addr, uint8_t* buffer, uint8_t nb)
{
	uint8_t addr = (start_addr | CC1101_READ_OFFSET | CC1101_BURST_MODE);
	return cc1101_spi_transfer(addr, NULL, buffer, nb);
}
/* Write single register value. Return the global status byte */
static uint8_t cc1101_write_reg(uint8_t addr, uint8_t val)
{
	return cc1101_spi_transfer((addr | CC1101_WRITE_OFFSET), &val, NULL, 1);
}
static uint8_t cc1101_write_burst_reg(uint8_t start_addr, uint8_t* buffer, uint8_t nb)
{
	uint8_t addr = (start_addr | CC1101_WRITE_OFFSET | CC1101_BURST_MODE);
	return cc1101_spi_transfer(addr, buffer, NULL, nb);
}


/* Send command and return global status byte */
static uint8_t cc1101_send_cmd(uint8_t addr)
{
	return cc1101_spi_transfer((addr | CC1101_WRITE_OFFSET), NULL, NULL, 0);
}


/***************************************************************************** */
/* SPI registers and commands access Wrappers */
void cc1101_reset(void)
{
	cc1101_send_cmd(CC1101_CMD(reset));
}
void cc1101_power_up_reset(void) __attribute__ ((alias ("cc1101_reset")));

void cc1101_flush_tx_fifo(void)
{
	/* Make sure that the radio is in IDLE state before flushing the FIFO */
	cc1101_send_cmd(CC1101_CMD(state_idle));
	cc1101_send_cmd(CC1101_CMD(flush_tx));
}
void cc1101_flush_rx_fifo(void)
{
	/* Make sure that the radio is in IDLE state before flushing the FIFO */
	cc1101_send_cmd(CC1101_CMD(state_idle));
	cc1101_send_cmd(CC1101_CMD(flush_rx));
}


/***************************************************************************** */
/* Read global status byte */
uint8_t cc1101_read_status(void)
{
	return cc1101_send_cmd(CC1101_CMD(no_op));
}
/* Read packet status byte */
uint8_t cc1101_read_pkt_status(void)
{
	return cc1101_send_cmd(CC1101_STATUS(packet_status));
}


/***************************************************************************** */
/* Change Current mode / status to RX */
void cc1101_enter_rx_mode(void)
{
	uint8_t status = (cc1101_read_status() & CC1101_STATE_MASK);
	if (status != CC1101_STATE_RX) {
		if ((status != CC1101_STATE_FSTON) && (status != CC1101_STATE_TX)) {
			cc1101_send_cmd(CC1101_CMD(state_idle));
		}
	}
	cc1101_send_cmd(CC1101_CMD(state_rx));
}

static uint8_t cc1101_enter_tx_mode(void)
{
	uint8_t status = (cc1101_read_status() & CC1101_STATE_MASK);
	if (status > CC1101_STATE_FSTON) {
			cc1101_send_cmd(CC1101_CMD(state_idle));
	}
	cc1101_send_cmd(CC1101_CMD(state_tx));

	/* Wait until chip is in Tx state */
	do {
		status = (cc1101_read_status() & CC1101_STATE_MASK);
		if (status == CC1101_STATE_TXFIFO_UNDERFLOW) {
			cc1101_flush_tx_fifo();
			return status;
		}
		if (status == CC1101_STATE_RXFIFO_OVERFLOW) {
			cc1101_flush_rx_fifo();
			return status;
		}
	} while (status != CC1101_STATE_TX);
	return 0;
}

void cc1101_enter_fstxon_state(void)
{
	uint8_t status = (cc1101_read_status() & CC1101_STATE_MASK);
	if (status != CC1101_STATE_FSTON) {
		if ((status != CC1101_STATE_TX) && (status != CC1101_STATE_RX)) {
			cc1101_send_cmd(CC1101_CMD(state_idle));
		}
		cc1101_send_cmd(CC1101_CMD(start_freq_synth));
	}
}


/***************************************************************************** */
/* Signal strength and link quality */

/* Return the signal strength indication based in the last packet received */
uint8_t cc1101_get_signal_strength_indication(void)
{
	uint8_t val = cc1101.rx_sig_strength;
	if (val >= 128) {
		val = 255 - val;
	}
	return (val >> 1) + 74;  /* val/2 + 74 */
}
/* Return the link quality indication based in the last packet received */
uint8_t cc1101_get_link_quality(void)
{
	return (0x3F - (cc1101.link_quality & 0x3F));
}


/* Request a calibration */
void cc1101_send_calibration_request(void)
{
	cc1101_send_cmd(CC1101_CMD(synth_calibration));
}

/***************************************************************************** */
/* Rx fifo state :
 * Return 0 when fifo is empty, or number of remaining bytes when non empty and no
 *    overflow occured.
 * Return -1 when an overflow occured.
 * Upon overflow, the radio is placed in idle state and the RX fifo flushed.
 * Else the radio is placed in RX state.
 */
int cc1101_rx_fifo_state(void)
{
	uint8_t rx_status = 0;

	cc1101_send_cmd(CC1101_CMD(state_rx));
	rx_status = cc1101_read_reg(CC1101_STATUS(rx_bytes));

	/* Check for buffer overflow */
	if (rx_status & CC1101_RX_FIFO_OVERFLOW) {
		cc1101_flush_rx_fifo();
		return -1;
	}
	return rx_status;
}

/* Tx fifo state :
 * Return 0 when fifo is empty, or number of remaining bytes when non empty and no
 *    overflow occured.
 * Return a negative value on error:
 *     when an underflow occured, return value is -1
 *     on other errors return value is (-1) * (global status byte)
 * Upon error, the radio is placed in idle state and the TX fifo flushed.
 * Else the radio is placed in TX state.
 */
int cc1101_tx_fifo_state(void)
{
	uint8_t tx_status = 0, ret = 0;

	ret = cc1101_enter_tx_mode();
	if (ret != 0) {
		return -ret;
	}
	tx_status = cc1101_read_reg(CC1101_STATUS(tx_bytes));

	/* Check for buffer  */
	if (tx_status & CC1101_TX_FIFO_UNDERFLOW) {
		cc1101_flush_tx_fifo();
		return -1;
	}
	return tx_status;
}

/***************************************************************************** */
/* Send packet
 * When using a packet oriented communication with packet size and address included
 *   in the packet, these must be included in the packet by the software before
 *   calling this function.
 * In order to send packets of more than 64 bytes (including length and address) the
 *   user will have to write his own function.
 */
int cc1101_send_packet(uint8_t* buffer, uint8_t size)
{
	uint8_t ret = 0;
	if (size > CC1101_FIFO_SIZE) {
		return -E2BIG;
	}
	cc1101_write_burst_reg(CC1101_FIFO_BURST, buffer, size);
	ret = cc1101_enter_tx_mode();
	if (ret != 0) {
		return -ret;
	}
	return size;
}

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
int cc1101_receive_packet(uint8_t* buffer, uint8_t size, uint8_t* status)
{
	uint8_t rx_status = 0;
	uint8_t read_size = 0, pkt_length = 0;
	int ret = 0;
	uint8_t st_buff[2];

	/* Get fifo state */
	rx_status = cc1101_read_reg(CC1101_STATUS(rx_bytes));
	if (status != NULL) {
		*status = rx_status;
	}

	/* Empty fifo ? */
	if (rx_status == 0) {
		return 0;
	}
	/* Receive one packet at most */
	pkt_length = cc1101_read_reg(CC1101_FIFO);
	if (pkt_length > (CC1101_FIFO_SIZE - 1)) {
		ret = -CC1101_ERR_RCV_PKT_TOO_BIG;
		if (size < (rx_status & CC1101_BYTES_IN_FIFO_MASK)) {
			read_size = size;
		} else {
			read_size = (rx_status & CC1101_BYTES_IN_FIFO_MASK);
		}
	} else if ((pkt_length + 1) > size) {
		ret = -CC1101_ERR_BUFF_TOO_SMALL;
		read_size = size;
	} else if (pkt_length > (rx_status & CC1101_BYTES_IN_FIFO_MASK)) {
		ret = -CC1101_ERR_INCOMPLET_PACKET;
		read_size = (rx_status & CC1101_BYTES_IN_FIFO_MASK);
	} else {
		ret = pkt_length + 1;
		read_size = pkt_length + 1;
	}
	/* Fill the buffer ! */
	buffer[0] = pkt_length;
	read_size--;
	cc1101_read_burst_reg(CC1101_FIFO_BURST, &(buffer[1]), read_size);

	/* Update the link quality and signal strength information */
	cc1101_read_burst_reg(CC1101_FIFO_BURST, st_buff, 2);
	cc1101.link_quality = st_buff[1];
	cc1101.rx_sig_strength  = st_buff[0];

	/* CRC error */
	if (!(cc1101.link_quality & CC1101_CRC_OK)) {
		cc1101_flush_rx_fifo();
		return -CC1101_ERR_CRC;
	}
	/* Overflow ? */
	if (rx_status & CC1101_RX_FIFO_OVERFLOW) {
		cc1101_flush_rx_fifo();
		return -CC1101_ERR_OVERFLOW;
	}
	return ret;
}

/***************************************************************************** */
/* CC1101 Initialisation */


/* Change the CC1101 address
 * Only packets addressed to the specified address (or broadcasted) will be
*    received.
 * Adresses 0x00 and 0xFF are broadcast
 * This function places the CC1101 chip in idle state.
 */
void cc1101_set_address(uint8_t address)
{
	cc1101_send_cmd(CC1101_CMD(state_idle));
	cc1101_write_reg(CC1101_REGS(device_addr), address);
}

/* Set current channel to use.
 * The caller is responsible for checking that the channel spacing and channel bandwith are configures
 * correctly to prevent overlaping channels, or to use only non-overlaping channel numbers.
 * This function places the CC1101 chip in idle state.
 */
void cc1101_set_channel(uint8_t chan)
{
	cc1101_send_cmd(CC1101_CMD(state_idle));
	cc1101_write_reg(CC1101_REGS(channel_number), chan);
}

/* Enter power down mode
 * Power down mode is exited by setting the chip select pin low (any access to the CC1101 will do so)
 */
void cc1101_power_down(void)
{
	cc1101_send_cmd(CC1101_CMD(state_idle));
	msleep(1);
	cc1101_send_cmd(CC1101_CMD(crystal_off));
	msleep(1);
	cc1101_send_cmd(CC1101_CMD(state_power_down));
}

/* Change a configuration byte.
 * This function places the CC1101 chip in idle state.
 */
void cc1101_set_config(uint8_t byte_addr, uint8_t value)
{
	cc1101_send_cmd(CC1101_CMD(state_idle));
	cc1101_write_reg(byte_addr, value);
}

/* Table of initial settings, in the form of address / value pairs. */
static uint8_t rf_init_settings[] = {
	CC1101_REGS(gdo_config[0]), 0x2E, /* GDO_2 - High impedance (Disabled) */
	CC1101_REGS(gdo_config[1]), 0x29, /* GDO_1 - Chip ready | Low drive strength */
	CC1101_REGS(gdo_config[2]), 0x07, /* GDO_0 - Assert on CRC OK | Disable temp sensor */

	/* RX FIFO and TX FIFO thresholds - 0x03 - FIFOTHR */
	CC1101_REGS(fifo_thresholds), 0x47, /* ADC_retention - Bytes in TX FIFO:33 - Bytes in RX FIFO:32 */
	/* Packet length - 0x06 - PKTLEN */
	CC1101_REGS(packet_length), 0x3F, /* Max packet length of 63 bytes */

	/* Packet automation control - 0x07 .. 0x08 - PKTCTRL1..0 */
	CC1101_REGS(pkt_ctrl[0]), 0x0F, /* Accept all sync, CRC err auto flush, Append, Addr check and Bcast */
	CC1101_REGS(pkt_ctrl[1]), 0x05, /* No data Whitening, Use fifos, CRC check, Variable pkt length */

	/* Device address - 0x09 - ADDR */
	CC1101_REGS(device_addr), 0x00, /* 0x00 and 0xFF are broadcast */
	/* Channel number - 0x0A - CHANNR */
	CC1101_REGS(channel_number), 0x00, /* Channel 0 */

	/* Frequency synthesizer control - 0x0B .. 0x0C - FSCTRL1..0 */
	CC1101_REGS(freq_synth_ctrl[0]), 0x0C, /* Used for ?? IF: 304.6875 KHz */
	CC1101_REGS(freq_synth_ctrl[1]), 0x00, /* Reset value */

	/* Carrier Frequency control - FREQ2..0 : Fcarrier == 867.999939 MHz */
	CC1101_REGS(freq_control[0]), 0x21, /* 0x216276 == Fcarrier * 2^16 / Fxtal */
	CC1101_REGS(freq_control[1]), 0x62, /*          == approx(868 MHz) * 65536 / 26 MHz */
	CC1101_REGS(freq_control[2]), 0x76,

	/* Modem configuration - MDMCFG4..0 - 0x10 .. 0x14 */
	/* MDMCFG4..3 : RX filterbandwidth = 541.666667 kHz and Datarate = 249.938965 kBaud */
	CC1101_REGS(modem_config[0]), 0x2D,
	CC1101_REGS(modem_config[1]), 0x3B,
	/* MDMCFG2 : 30/32 sync word bits + sensitivity, Manchester disabled, GFSK, Digital DC filter enabled */
	CC1101_REGS(modem_config[2]), 0x13,
	/* MDMCFG1..0 : FEC disabled, 4 preamble bytes, Channel spacing = 199.951172 kHz */
	CC1101_REGS(modem_config[3]), 0x22,
	CC1101_REGS(modem_config[4]), 0xF8,
	/* Modem deviation : DEVIATN */
	CC1101_REGS(modem_deviation), 0x62, /* Deviation = 127 kHz */


	/* Main Radio Control State Machine Configuration - MCSM2..0 - 0x16 .. 0x18 */
	CC1101_REGS(radio_stm[0]), 0x07, /* Use default */
	CC1101_REGS(radio_stm[1]), 0x3F, /* CCA mode if RSSI below threshold unless Rx, Stay in RX, Go to RX */
	CC1101_REGS(radio_stm[2]), 0x18, /* PO timeout 149-155us, Auto calibrate from idle to rx/tx */

	/* Frequency Offset Compensation configuration - FOCCFG */
	CC1101_REGS(freq_offset_comp), 0x1D, /* 4K before, K/2 after, BWchan/8 */
	CC1101_REGS(bit_sync), 0x1C, /* 1*Ki, 2*Kp, Ki/2, Kp, +0 (no data rate compensation) */

	/* AGC control - 0x1B .. 0x1D - AGCCTRL2..0 */
	CC1101_REGS(agc_ctrl[0]), 0xC7, /* Don't use 3highest gain, Max LNA gain, 42dB amp from chan filter */
	CC1101_REGS(agc_ctrl[1]), 0x00, /* LNA 2 gain decr first, Carrier sense relative threshold disabled */
	CC1101_REGS(agc_ctrl[2]), 0xB0, /* Medium settings, 24 samples wait time, normal op. */

	/* Wake on radio : use defaults */

	/* Front End Rx/Tx config */
	CC1101_REGS(front_end_rx_cfg), 0xB6,
	CC1101_REGS(front_end_tx_cfg), 0x10,

	/* Frequency synthesizer calibration - 0x23 .. 0x26 - FSCAL3..0 */
	CC1101_REGS(freq_synth_cal[0]), 0xEA,
	CC1101_REGS(freq_synth_cal[1]), 0x2A,
	CC1101_REGS(freq_synth_cal[2]), 0x00,
	CC1101_REGS(freq_synth_cal[3]), 0x1F,
	/* Frequency synthesizer calibration control - 0x29 - FSTEST */
	CC1101_REGS(freq_synth_cal_ctrl), 0x59,

	/* Various test settings - 0x2C .. 0x2E - TEST2..0 */
//	CC1101_REGS(test[0]), 0x88,
//	CC1101_REGS(test[1]), 0x31,
	CC1101_REGS(test[2]), 0x09, /* Disable VCO selection calibration stage */
};

static uint8_t paTable[] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/* Configure pins, reset the CC1101, and put the CC1101 chip in idle state */
void cc1101_init(uint8_t ssp_num, const struct pio* cs_pin, const struct pio* miso_pin)
{
	if ((cs_pin == NULL) || (miso_pin == NULL)) {
		return;
	}

	cc1101.spi_num = ssp_num;
	/* Copy CS pin info and configure pin as GPIO output */
	pio_copy(&(cc1101.cs_pin), cs_pin);
	/* Configure CS as output and set high */
	config_gpio(cs_pin, LPC_IO_DIGITAL, GPIO_DIR_OUT, 1);

	/* Copy MISO pin info (no config, should be already configured as SPI) */
	pio_copy(&(cc1101.miso_pin), miso_pin);

	cc1101_power_up_reset();
	cc1101_send_cmd(CC1101_CMD(state_idle));
}

/* Write / send all the configuration register values to the CC1101 chip */
void cc1101_config(void)
{
	int i = 0;
	cc1101_send_cmd(CC1101_CMD(state_idle));
	/* Write RF initial settings to CC1101 */
	for (i = 0; i < sizeof(rf_init_settings); i += 2) {
		cc1101_write_reg(rf_init_settings[i], rf_init_settings[i + 1]);
	}
	/* Write PA Table value */
	cc1101_write_reg(CC1101_PATABLE, paTable[0]);
}

/* Update CC1101 config
 * Arguments are the settings table which is a table of address and value pairs,
 *   and the table length, which must be even.
 * Puts the CC1101 chip in idle state
 */
void cc1101_update_config(uint8_t* settings, uint8_t len)
{
	int i = 0;
	if (len & 0x01) {
		return;
	}
	/* Chip must be in idle state when modifying any of the Frequency or channel registers.
	 * Move to idle state for all cases, easier. */
	cc1101_send_cmd(CC1101_CMD(state_idle));
	for (i = 0; i < len; i += 2) {
		cc1101_write_reg(settings[i], settings[i + 1]);
	}
}

/* Change PA Table value */
void cc1101_set_patable(uint8_t val)
{
	cc1101_write_reg(CC1101_PATABLE, val);
}

