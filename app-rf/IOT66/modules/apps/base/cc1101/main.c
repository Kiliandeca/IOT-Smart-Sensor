/****************************************************************************
 *   apps/base/cc1101/main.c
 *
 * CC1101 example
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
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


#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "drivers/ssp.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "lib/stdio.h"
#include "extdrv/status_led.h"
#include "extdrv/cc1101.h"


#define MODULE_VERSION    0x04
#define MODULE_NAME "GPIO Demo Module"


#define SELECTED_FREQ  FREQ_SEL_48MHz

/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* UART 1 */
	{ LPC_UART1_RX_PIO_0_8, LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9, LPC_IO_DIGITAL },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};

const struct pio cc1101_cs_pin = LPC_GPIO_0_15;
const struct pio cc1101_miso_pin = LPC_SSP0_MISO_PIO_0_16;

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_brown_out_detection_config(0); /* No ADC used */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	status_led_config(&status_led_green, &status_led_red);
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	uprintf(UART1, name);
	while (1);
}


/***************************************************************************** */
/* Data sent on radio comes from the UART, put any data received from UART in
 * cc_tx_buff and send when either '\r' or '\n' is received.
 * This function is very simple and data received between cc_tx flag set and
 * cc_ptr rewind to 0 may be lost. */
#define RF_BUFF_LEN  60
static volatile uint32_t cc_tx = 0;
static volatile uint8_t cc_tx_buff[RF_BUFF_LEN];
static volatile uint8_t cc_ptr = 0;
void cc1101_test_rf_serial_link_tx(uint8_t c)
{
	if (cc_ptr < RF_BUFF_LEN) {
		cc_tx_buff[cc_ptr++] = c;
	} else {
		cc_ptr = 0;
	}
	if ((c == '\n') || (c == '\r')) {
		cc_tx = 1;
	}
}

/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, cc1101_test_rf_serial_link_tx);
	uart_on(UART1, 115200, NULL);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */

	/* Radio */
	cc1101_init(0, &cc1101_cs_pin, &cc1101_miso_pin); /* ssp_num, cs_pin, miso_pin */
	cc1101_config();

#define BUFF_LEN 60
	while (1) {
		char buff[BUFF_LEN];
		int len = 0;
		chenillard(25);

		/* Any Data to send */
		if (cc_tx) {
			uint8_t cc_tx_data[RF_BUFF_LEN + 2];
			uint8_t tx_len = cc_ptr;
			uint8_t val = 0;
			int ret = 0;
			/* Create a local copy */
			memcpy((char*)&(cc_tx_data[2]), (char*)cc_tx_buff, tx_len);
			/* "Free" the rx buffer as soon as possible */
			cc_ptr = 0;
			/* Prepare buffer for sending */
			cc_tx_data[0] = tx_len + 1;
			cc_tx_data[1] = 0; /* Broadcast */
			/* Send */
			ret = cc1101_send_packet(cc_tx_data, (tx_len + 2));
			/* Give some feedback on UART 1 */
			uprintf(UART1, "Tx ret: %d\n", ret);
			/* Wait a short time for data to be sent ... */
			/* FIXME : This should be done using the packet sent signal from CC1101 on GDO pin */
			msleep(2);
			do {
				ret = cc1101_tx_fifo_state();
				if (ret < 0) {
					uprintf(UART1, "Tx Underflow !\n");
					break;
				}
			} while (ret != 0);
			/* Get back to Receiver mode */
			cc1101_enter_rx_mode();
			/* And give some feedback again */
			val = cc1101_read_status();
			uprintf(UART1, "Status : 0x%02x, Sending : %d\n", val, tx_len);
			serial_write(UART1, (char*)&(cc_tx_data[2]), tx_len);
			cc_tx = 0;
		}

		/* Check for received data on RF link */
		if (1) {
			int ret = 0, rxlen = 0, addr = 0;
			uint8_t status = 0;
			/* Check for received packet (and get it if any) */
			ret = cc1101_receive_packet((uint8_t*)buff, 50, &status);
			cc1101_enter_rx_mode(); /* We should already be in receive mode. */
			/* >0 means we got something, <0 is an error, =0 is no packet received */
			if (ret > 0) {
				uint8_t val = 0;
				len = ret;
				/* Send packet on UART 0 */
				serial_write(UART0, &buff[2], (len - 2));
				uprintf(UART0, "\n");
				/* And also some feedback on UART 1 */
				rxlen = buff[0];
				addr = buff[1];
				val = cc1101_get_signal_strength_indication();
				uprintf(UART1, "Status: %d, link: %d, len: %d/%d, addr: %d\n", status, val, rxlen, len, addr);
			} else if (ret < 0) {
				/* Send error on UART 1 */
				uprintf(UART1, "Rx Error: %d, len: %d\n", -ret, (status & 0x7F));
			}
		}
	}
	return 0;
}




