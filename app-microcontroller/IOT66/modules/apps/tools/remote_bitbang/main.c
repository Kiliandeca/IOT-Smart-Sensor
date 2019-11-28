/****************************************************************************
 *   apps/tools/remote_bitbang/main.c
 *
 *  Support for remote bitbang SWD access using openOCD
 *
 * Copyright 2015 Cyprien Laplace <cyprien@cypou.net>
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


/*
 * This app provides an SWD bridge for remote bitbang SWD access using openocd.
 * This allows remote debugging using gdb.
 *
 * Refer to README file for usage informations.
 */

#include "core/system.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/status_led.h"


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
	{ LPC_UART1_RX_PIO_0_8,  LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9,  LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};


const struct pio led_green = LPC_GPIO_0_28;
const struct pio led_red = LPC_GPIO_0_29;
#define LED_RED    (1 << led_red.pin)
#define LED_GREEN  (1 << led_green.pin)

/* These two MUST be on the same port */
const struct pio swd_io  = LPC_GPIO_0_23;
const struct pio swd_clk = LPC_GPIO_0_24;
#define SWD_IO   (1 << swd_io.pin)
#define SWD_CLK  (1 << swd_clk.pin)

void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_brown_out_detection_config(0); /* No ADC used */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	status_led_config(&led_green, &led_red);
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler 
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	uprintf(UART0, name);
	while (1);
}

static void uart_cb(uint8_t data)
{
	char in;
	struct lpc_gpio* gpio_io  = LPC_GPIO_REGS(swd_io.port);
	struct lpc_gpio* gpio_red = LPC_GPIO_REGS(led_red.port);
	struct lpc_gpio* gpio_green = LPC_GPIO_REGS(led_red.port);

	if (data >= '0' && data <= '7') {
		uint32_t val = 0;
		data -= '0';
		if (data & 1)
			val |= SWD_IO;
		if (data & 4)
			val |= SWD_CLK;
		gpio_io->out = val;
	} else switch(data) {
		case 'i':
			gpio_io->data_dir &= ~SWD_IO;
			break;
		case 'o':
			gpio_io->data_dir |= SWD_IO;
			break;
		case 'R':
			in = '0' + !!(gpio_io->in & SWD_IO);
			serial_send_quickbyte(UART0, in);
			gpio_green->out ^= LED_GREEN;
			break;
		default:
			gpio_red->out ^= LED_RED;
	}
}

/***************************************************************************** */
int main(void)
{
	system_init();
	config_gpio(&swd_io,  LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 1);
	config_gpio(&swd_clk, LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 1);
	uart_on(UART0, 115200, uart_cb);
	status_led(red_on);
	while(1);
	return 0;
}
