/****************************************************************************
 *  drivers/ssp.c
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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



/***************************************************************************** */
/*                SSP                                                          */
/***************************************************************************** */

/* SSP/SPI driver for the SSP bus integrated module of the LPC122x.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */

#include "core/system.h"
#include "core/pio.h"
#include "lib/errno.h"
#include "lib/string.h"
#include "drivers/ssp.h"


struct ssp_device
{
	uint32_t num;
	struct lpc_ssp* regs;
	uint32_t current_clk;
	uint32_t mutex;
	volatile uint32_t int_rx_stats;
	volatile uint32_t int_overrun_stats;
	volatile uint32_t int_rx_timeout_stats;

	uint32_t irq;
	uint32_t clk_ctrl_bit;
};

#define NUM_SSPS 1
static struct ssp_device ssps[NUM_SSPS] = {
	{
		.num = 0,
		.regs = LPC_SSP0,
		.current_clk = 0,
		.mutex = 0,
		.int_rx_stats = 0,
		.int_overrun_stats = 0,
		.int_rx_timeout_stats = 0,
		.irq = SSP0_IRQ,
		.clk_ctrl_bit = LPC_SYS_ABH_CLK_CTRL_SSP0,
	},
};



/* Handlers */
void SSP_0_Handler(void)
{
	struct lpc_ssp* ssp = LPC_SSP0;
	uint32_t intr_flags = ssp->masked_int_status;

	/* Clear the interrupts. Other bits are cleared by fifo access */
	ssp->int_clear = (intr_flags & (LPC_SSP_INTR_RX_OVERRUN | LPC_SSP_INTR_RX_TIMEOUT));
	if (intr_flags & LPC_SSP_INTR_RX_OVERRUN) {
		ssps[0].int_overrun_stats += 1;
	}
	if (intr_flags & LPC_SSP_INTR_RX_TIMEOUT) {
		ssps[0].int_rx_timeout_stats += 1;
	}
}



/***************************************************************************** */
/* SPI Bus mutex */

#if MULTITASKING == 1
int spi_get_mutex(uint8_t ssp_num)
{
	/* Note : a multitasking OS should call the scheduler here. Any other system
	 *   will get frozen, unless some interrupt routine has been set to release
	 *   the mutex.
	 */
	do {} while (sync_lock_test_and_set(&(ssps[ssp_num].mutex), 1) == 1);
	return 1;
}
#else
int spi_get_mutex(uint8_t ssp_num)
{
	if (sync_lock_test_and_set(&(ssps[ssp_num].mutex), 1) == 1) {
		return -EBUSY;
	}
	return 1;
}
#endif
void spi_release_mutex(uint8_t ssp_num)
{
	sync_lock_release(&(ssps[ssp_num].mutex));
}


/***************************************************************************** */
/* This function is used to transfer a word (4 to 16 bits) to AND from a device
 * As the SPI bus is full duplex, data can flow in both directions, but the clock is
 *   always provided by the master. The SPI clock cannont be activated without sending
 *   data, which means we must send data to the device when we want to read data from
 *   the device.
 * Note : the SSP device number is not checked, thus a value above the number of SSP
 *   devices present on the micro-controller may break your programm.
 *   Double check the value of the ssp_num parameter. The check is made on the call to
 *   ssp_master_on() or ssp_slave_on().
 * This function does not take care of the SPI chip select.
 */
uint16_t spi_transfer_single_frame(uint8_t ssp_num, uint16_t data)
{
	struct lpc_ssp* ssp_regs = ssps[ssp_num].regs;
	ssp_regs->data = data;
	/* Wait until the busy bit is cleared */
	while (ssp_regs->status & LPC_SSP_ST_BUSY);
	return ssp_regs->data;
}



/***************************************************************************** */
/* Multiple words (4 to 16 bits) transfer function on the SPI bus. */

/* Internal function used to send 9 to 16 bits wide data */
static int spi_transfer_words(struct lpc_ssp* ssp, uint16_t* data_out, uint16_t* data_in, int size)
{
	int count = 0;
	uint16_t data_read = 0; /* Used to store SPI Rx data */

	/* Transfer */
	do {
		/* Fill Tx fifo with available data, but stop if rx fifo is full */
		while ((count < size) &&
			  ((ssp->status & (LPC_SSP_ST_TX_NOT_FULL | LPC_SSP_ST_RX_FULL)) == LPC_SSP_ST_TX_NOT_FULL)) {
			ssp->data = *data_out;
			count++;
			data_out++;
		}

		/* Read some of the replies, but stop if there's still data to send and the fifo
		 *  is running short */
		while ((ssp->status & LPC_SSP_ST_RX_NOT_EMPTY) &&
				!((count < size) && (ssp->raw_int_status & LPC_SSP_INTR_TX_HALF_EMPTY))) {
			/* Read the data (mandatory) */
			data_read = ssp->data;
			if (data_in != NULL) {
				/* And store when requested */
				*data_in = data_read;
				data_in++;
			}
		}
	/* Go on till both all data is sent and all data is received */
	} while ((count < size) || (ssp->status & (LPC_SSP_ST_BUSY | LPC_SSP_ST_RX_NOT_EMPTY)));

	return count;
}
/* Internal function used to send 4 to 8 bits wide data */
static int spi_transfer_bytes(struct lpc_ssp* ssp, uint8_t* data_out, uint8_t* data_in, int size)
{
	int count = 0;
	uint8_t data_read = 0; /* Used to store SPI Rx data */

	/* Transfer */
	do {
		/* Fill Tx fifo with available data, but stop if rx fifo is full */
		while ((count < size) &&
			  ((ssp->status & (LPC_SSP_ST_TX_NOT_FULL | LPC_SSP_ST_RX_FULL)) == LPC_SSP_ST_TX_NOT_FULL)) {
			ssp->data = (uint16_t)(*data_out);
			count++;
			data_out++;
		}

		/* Read some of the replies, but stop if there's still data to send and the fifo
		 *  is running short */
		while ((ssp->status & LPC_SSP_ST_RX_FULL) ||
                ((ssp->status & LPC_SSP_ST_RX_NOT_EMPTY) &&
				 !((count < size) && (ssp->raw_int_status & LPC_SSP_INTR_TX_HALF_EMPTY)))) {
			/* Read the data (mandatory) */
			data_read = (uint8_t)ssp->data;
			if (data_in != NULL) {
				/* And store when requested */
				*data_in = data_read;
				data_in++;
			}
		}
	/* Go on till both all data is sent and all data is received */
	} while ((count < size) || (ssp->status & (LPC_SSP_ST_BUSY | LPC_SSP_ST_RX_NOT_EMPTY)));

	return count;
}


/* Multiple words (4 to 16 bits) transfer function on the SPI bus.
 * The SSP fifo is used to leave as little time between frames as possible.
 * Parameters :
 *  ssp_num : ssp device number. The SSP device number is not checked, thus a value above
 *            the number of SSP devices present on the micro-controller may break your
 *            programm. Double check the value of the ssp_num parameter. The check is made
 *            on the call to ssp_master_on() or ssp_slave_on().
 *  size is the number of frames, each one having the configured data width (4 to 16 bits).
 *  data_out : data to be sent. Data will be read in the lower bits of the 16 bits values
 *             pointed by "data_out" for each frame. If NULL, then the content of data_in
 *             will be used.
 *  data_in : buffer for read data. If NULL, read data will be discarded.
 * Returns the number of data words transfered or negative value on error.
 * As the SPI bus is full duplex, data can flow in both directions, but the clock is
 *   always provided by the master. The SPI clock cannont be activated without sending
 *   data, which means we must send data to the device when we want to read data from
 *   the device.
 * This function does not take care of the SPI chip select.
 * Note: there's no need to count Rx data as it is equal to Tx data
 * Note : the SSP device number is not checked, thus a value above the number of SSP
 *   devices present on the micro-controller may break your programm.
 *   Double check the value of the ssp_num parameter. The check is made on the call to
 *   ssp_master_on() or ssp_slave_on().
 */
int spi_transfer_multiple_frames(uint8_t ssp_num, void* data_out, void* data_in, int size, int width)
{
	struct lpc_ssp* ssp_regs = ssps[ssp_num].regs;

	/* Did the user provide a buffer to send data ? */
	if (data_out == NULL) {
		if (data_in == NULL) {
			return -EINVAL;
		}
		data_out = data_in;
	}
	/* Transfer using either byte or word transfer functions */
	if (width > 8) {
		return spi_transfer_words(ssp_regs, (uint16_t*)data_out, (uint16_t*)data_in, size);
	} else {
		return spi_transfer_bytes(ssp_regs, (uint8_t*)data_out, (uint8_t*)data_in, size);
	}
}



/***************************************************************************** */
uint32_t ssp_clk_on(uint8_t ssp_num, uint32_t rate)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct lpc_ssp* ssp_regs = ssps[ssp_num].regs;
	uint32_t prescale = 0, pclk_div = 0;
	uint32_t pclk = 0, div = 0;

	/* Do not divide by 0 */
	if (rate == 0) {
		return 0;
	}

	pclk = get_main_clock();

	/* Make sure we can get this clock rate */
	/* NOTE: We use only to divisors, so we could achieve lower clock rates by using
	 *   the third one. Any need for this though ?
	 */
	if ((pclk / rate) > 0xFFF0) {
		/* What should we do ? */
		div = 0xFFF0;
	} else {
		div = pclk / rate;
	}

	do {
		prescale += 2; /* Minimum value is 2, and must be even */
		pclk_div = (div / prescale);
	} while ((prescale > 0xFF) || (pclk_div > 0xFF));

	/* Activate the SSP clock (maybe divide main clock) */
	switch (ssp_num) {
		case 0:
			sys_config->ssp0_clk_div = pclk_div;
			break;
	}

	/* Set the prescaler */
	ssp_regs->clk_prescale = prescale;

	/* And return the achieved clock */
	return (pclk / (prescale * pclk_div));
}

void ssp_clk_update(void)
{
	int i = 0;
	for (i = 0; i < NUM_SSPS; i++) {
		if (ssps[i].current_clk) {
			ssp_clk_on(i, ssps[i].current_clk);
		}
	}
}


/***************************************************************************** */
/* SSP Setup as master */
/* Returns 0 on success
 * Parameters :
 *  frame_type is SPI, TI or MICROWIRE (use apropriate defines for this one:
 *     LPC_SSP_FRAME_SPI - LPC_SSP_FRAME_TI - LPC_SSP_FRAME_MICROWIRE).
 *  data_width is a number between 4 and 16.
 *  rate : The bit rate, in Hz.
 * The SPI Chip Select is not handled by the SPI driver in master SPI mode as it's
 *   handling highly depends on the device on the other end of the wires. Use a GPIO
 *   to activate the device's chip select (usually active low)
 */
int ssp_master_on(uint8_t ssp_num, uint8_t frame_type, uint8_t data_width, uint32_t rate)
{
	struct ssp_device* ssp = NULL;
	struct lpc_ssp* ssp_regs = NULL;

	if (ssp_num >= NUM_SSPS)
		return -EINVAL;
	ssp = &ssps[ssp_num];
	ssp_regs = ssp->regs;

	NVIC_DisableIRQ(ssp->irq);

	/* Power up the ssp block */
	subsystem_power(ssp->clk_ctrl_bit, 1);

	/* Configure the SSP mode */
	ssp_regs->ctrl_0 = (LPC_SSP_DATA_WIDTH(data_width) | frame_type | LPC_SSP_SPI_CLK_LOW | LPC_SSP_SPI_CLK_FIRST);
	ssp_regs->ctrl_1 = LPC_SSP_MASTER_MODE;

	/* Configure the clock : done after basic configuration */
	ssp->current_clk = ssp_clk_on(ssp_num, rate);

	/* Enable SSP */
	ssp_regs->ctrl_1 |= LPC_SSP_ENABLE;

	NVIC_EnableIRQ(ssp->irq);
	return 0; /* Config OK */
}

int ssp_slave_on(uint8_t ssp_num, uint8_t frame_type, uint8_t data_width, uint8_t out_en, uint32_t max_rate)
{
	struct ssp_device* ssp = NULL;
	struct lpc_ssp* ssp_regs = NULL;

	if (ssp_num >= NUM_SSPS)
		return -EINVAL;
	ssp = &ssps[ssp_num];
	ssp_regs = ssp->regs;

	NVIC_DisableIRQ(ssp->irq);
	/* Power up the ssp block */
	subsystem_power(ssp->clk_ctrl_bit, 1);

	/* Configure the SSP mode */
	ssp_regs->ctrl_0 = LPC_SSP_DATA_WIDTH(data_width);
	ssp_regs->ctrl_0 |= (frame_type | LPC_SSP_SPI_CLK_LOW | LPC_SSP_SPI_CLK_FIRST);
	ssp_regs->ctrl_1 = LPC_SSP_SLAVE_MODE;
	if (!out_en) {
		ssp_regs->ctrl_1 |= LPC_SSP_SLAVE_OUT_DISABLE;
	}

	/* Configure the clock : done after basic configuration.
	 * Our clock must be at least 12 times the master clock */
	ssp->current_clk = ssp_clk_on(ssp_num, max_rate * 16);

	/* Enable SSP */
	ssp_regs->ctrl_1 |= LPC_SSP_ENABLE;

	NVIC_EnableIRQ(ssp->irq);
	return 0; /* Config OK */
}

/* Turn off the SSP block */
void ssp_off(uint8_t ssp_num)
{
	struct ssp_device* ssp = NULL;

	if (ssp_num >= NUM_SSPS)
		return;
	ssp = &ssps[ssp_num];

	ssp->current_clk = 0;
	NVIC_DisableIRQ(ssp->irq);
	subsystem_power(ssp->clk_ctrl_bit, 0);

	/* Can be done even if we don't hold the mutex */
	sync_lock_release(&ssp->mutex);
}

