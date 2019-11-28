/****************************************************************************
 *  drivers/ssp.h
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

#ifndef DRIVERS_SSP_H
#define DRIVERS_SSP_H


/***************************************************************************** */
/*                SSP                                                          */
/***************************************************************************** */

/* SSP/SPI driver for the SSP bus integrated module of the LPC122x.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */

#include "lib/stdint.h"
#include "core/lpc_regs.h"


enum ssp_bus_number {
	SSP_BUS_0 = 0,
};

/* Set this to 1 for use of this driver in a multitasking OS, it will activate the SPI Mutex */
#define MULTITASKING 0




/***************************************************************************** */
/* SPI mutex for SPI bus */
/* In multitasking environment spi_get_mutex will block until mutex is available and always
 *  return 1. In non multitasking environments spi_get_mutex returns either 1 (got mutex)
 *  or -EBUSY (SPI bus in use)
 */
int spi_get_mutex(uint8_t ssp_num);
void spi_release_mutex(uint8_t ssp_num);


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
uint16_t spi_transfer_single_frame(uint8_t ssp_num, uint16_t data);


/***************************************************************************** */
/* Multiple words (4 to 16 bits) transfer function on the SPI bus.
 * The SSP fifo is used to leave as little time between frames as possible.
 * Parameters :
 *  ssp_num : ssp device number. The SSP device number is not checked, thus a value above
 *            the number of SSP devices present on the micro-controller may break your
 *            programm. Double check the value of the ssp_num parameter. The check is made
 *            on the call to ssp_master_on() or ssp_slave_on().
 *  size : the number of frames, each one having the configured data width (4 to 16 bits).
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
 */
int spi_transfer_multiple_frames(uint8_t ssp_num, void* data_out, void* data_in, int size, int width);



/***************************************************************************** */
void ssp_clk_update(void);


/***************************************************************************** */
/* SSP Setup as master */
/* Returns 0 on success
 * Parameters :
 *  frame_type is SPI, TI or MICROWIRE (use apropriate defines for this one).
 *     LPC_SSP_FRAME_SPI - LPC_SSP_FRAME_TI - LPC_SSP_FRAME_MICROWIRE).
 *  data_width is a number between 4 and 16.
 *  rate : The bit rate, in Hz.
 * The SPI Chip Select is not handled by the SPI driver in master SPI mode as it's
 *   handling highly depends on the device on the other end of the wires. Use a GPIO
 *   to activate the device's chip select (usually active low)
 */
int ssp_master_on(uint8_t ssp_num, uint8_t frame_type, uint8_t data_width, uint32_t rate);

int ssp_slave_on(uint8_t ssp_num, uint8_t frame_type, uint8_t data_width, uint8_t out_en, uint32_t max_rate);

/* Turn off given SSP block */
void ssp_off(uint8_t ssp_num);




/***************************************************************************** */
/*                     Synchronous Serial Communication                        */
/***************************************************************************** */
/* Synchronous Serial Communication (SSP) */
struct lpc_ssp
{
	volatile uint32_t ctrl_0;        /* 0x000 : Control Register 0 (R/W) */
	volatile uint32_t ctrl_1;        /* 0x004 : Control Register 1 (R/W) */
	volatile uint32_t data;          /* 0x008 : Data Register (R/W) */
	volatile const uint32_t status;  /* 0x00C : Status Registe (R/-) */
	volatile uint32_t clk_prescale;  /* 0x010 : Clock Prescale Register (R/W) */
	volatile uint32_t int_mask;      /* 0x014 : Interrupt Mask Set and Clear Register (R/W) */
	volatile uint32_t raw_int_status;     /* 0x018 : Raw Interrupt Status Register (R/-) */
	volatile uint32_t masked_int_status;  /* 0x01C : Masked Interrupt Status Register (R/-) */
	volatile uint32_t int_clear;     /* 0x020 : SSPICR Interrupt Clear Register (-/W) */
	volatile uint32_t dma_ctrl;      /* 0x024 : DMA Control Register (R/W) */
};
#define LPC_SSP0        ((struct lpc_ssp *) LPC_SSP0_BASE)

/* SSP Control 0 register */
#define LPC_SSP_DATA_WIDTH(x)    (((x) - 1) & 0x0F) /* Use 4 for 4 bits, 5 for 5 bits, .... */
#define LPC_SSP_FRAME_SPI        (0x00 << 4)
#define LPC_SSP_FRAME_TI         (0x01 << 4)
#define LPC_SSP_FRAME_MICROWIRE  (0x02 << 4)
#define LPC_SSP_SPI_CLK_LOW      (0x00 << 6)
#define LPC_SSP_SPI_CLK_HIGH     (0x01 << 6)
#define LPC_SSP_SPI_CLK_FIRST    (0x00 << 7)
#define LPC_SSP_SPI_CLK_LAST     (0x01 << 7)
#define LPC_SSP_SPI_CLK_RATE_DIV(x) (((x) & 0xFF) << 8)
/* SSP Control 1 register */
#define LPC_SSP_LOOPBACK_MODE      (0x01 << 0)
#define LPC_SSP_ENABLE             (0x01 << 1)
#define LPC_SSP_MASTER_MODE        (0x00 << 2)
#define LPC_SSP_SLAVE_MODE         (0x01 << 2)
#define LPC_SSP_SLAVE_OUT_DISABLE  (0x01 << 3)

/* SSP Status register */
#define LPC_SSP_ST_TX_EMPTY      (0x01 << 0)
#define LPC_SSP_ST_TX_NOT_FULL   (0x01 << 1)
#define LPC_SSP_ST_RX_NOT_EMPTY  (0x01 << 2)
#define LPC_SSP_ST_RX_FULL       (0x01 << 3)
#define LPC_SSP_ST_BUSY          (0x01 << 4)

/* SSP Interrupt Mask, Raw, Status, Clear */
#define LPC_SSP_INTR_RX_OVERRUN      (0x01 << 0)
#define LPC_SSP_INTR_RX_TIMEOUT      (0x01 << 1)
#define LPC_SSP_INTR_RX_HALF_FULL    (0x01 << 2)
#define LPC_SSP_INTR_TX_HALF_EMPTY   (0x01 << 3)

/* SSP DMA Control */
#define LPC_SSP_RX_DMA_EN   (0x01 << 0)
#define LPC_SSP_TX_DMA_EN   (0x01 << 1)


#endif /* DRIVERS_SSP_H */

