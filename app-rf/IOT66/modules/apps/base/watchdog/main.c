/****************************************************************************
 *   apps/base/watchdog/main.c
 *
 * Watchdog example
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
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/status_led.h"
#include "drivers/adc.h"


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
	{ LPC_UART1_RX_PIO_0_8,  LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9,  LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};

const struct pio_config adc_pins[] = {
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	{ LPC_ADC_AD3_PIO_1_1,  LPC_IO_ANALOG },
	{ LPC_ADC_AD4_PIO_1_2,  LPC_IO_ANALOG },
	{ LPC_ADC_AD5_PIO_1_3,  LPC_IO_ANALOG },
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;


static volatile int got_wdt_int = 0;
void tmp_callback(void)
{
	got_wdt_int = 1;
}


const struct wdt_config wdconf = {
	.clk_sel = WDT_CLK_IRC,
	.intr_mode_only = 0,
	.callback = tmp_callback,
	.locks = 0,
	.nb_clk = 0x4FFFFF,
	.wdt_window = 0,
	.wdt_warn = 0x3FF,
};

/***************************************************************************** */
/* This will display the integer value read on the ADC, between 0 and 1024.
 * ADC must be initialised prior to calls to adc_display() (it means that adc_on(NULL)
 *    must be called before using this function.
 * adc_num is an ADC channel number (integer between 0 and 7)
 *    use LPC_ADC(x) for channel selection.
 * returns ADC convertion value or negative value on error.
 */
int adc_display(int adc_num, int uart_num)
{
	uint16_t val = 0;
	int ret = 0;

	adc_start_convertion_once(adc_num, LPC_ADC_SEQ(0), 0);
	msleep(10);
	ret = adc_get_value(&val, adc_num);
	if (ret < 0) {
		return ret;
	} else {
		uprintf(uart_num, "ADC(%d): %d (raw: 0x%04x)\r\n", adc_num, val, val);
	}
	return val;
}


/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	set_pins(adc_pins);
	gpio_on();
	status_led_config(&status_led_green, &status_led_red);
	/* System tick timer MUST be configured and running in order to use the sleeping functions */
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
	uprintf(UART0, name);
	while (1);
}



/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, NULL);
	uart_on(UART1, 115200, NULL);
	adc_on(NULL);

	uprintf(UART1, "System started\n");
	uprintf(UART0, "System started\n");
	msleep(5);
	watchdog_config(&wdconf);
	uprintf(UART0, "Watchdog started\n");
	uprintf(UART1, "Watchdog started\n");

	while (1) {
		watchdog_feed();
		chenillard(50);
		/* ADC Test */
		adc_display(LPC_ADC(0), 0);
		adc_display(LPC_ADC(0), 1);
		if (got_wdt_int != 0) {
			uprintf(UART1, "Watchdog intr !\n");
			watchdog_feed();
			got_wdt_int = 0;
		}
	}
	return 0;
}



