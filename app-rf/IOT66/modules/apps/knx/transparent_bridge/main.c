/****************************************************************************
 *   apps/knx/transparent_bridge/main.c
 *
 * KNX support example
 *
 * Copyright 2015 Nathael Pajani <nathael.pajani@ed3l.fr>
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
#include "extdrv/ncn5120.h"


#define MODULE_VERSION    0x04
#define MODULE_NAME "KNX Module"

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
	/* UART 1 (KNX) */
	{ LPC_UART1_RX_PIO_0_8,  LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9,  LPC_IO_DIGITAL },
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* ADC */
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	/* Timer 3 */
#define LPC_TIMER_PIN_CONFIG   (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL | LPC_IO_DRIVE_HIGHCURENT)
	{ LPC_TIMER_32B1_M0_PIO_0_23, LPC_TIMER_PIN_CONFIG },
	{ LPC_TIMER_32B1_M1_PIO_0_24, LPC_TIMER_PIN_CONFIG },
	/* User GPIO */
	{ LPC_GPIO_0_0, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_25, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_26, LPC_IO_DIGITAL },
	/* KNX GPIO */
	{ LPC_GPIO_0_6, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_7, LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};

/*
 * RESETB and SAVEB pin
 *  The RESETB signal can be used to keep the host controller in a reset state. When RESETB is low this
 *  indicates that the bus voltage is too low for normal operation and that the fixed DC−DC converter
 *  has not started up. It could also indicate a Thermal Shutdown (TSD). The RESETB signal also indicates
 *  if communication between host and NCN5120 is possible.
 *
 *  The SAVEB signal indicates correct operation. When SAVEB goes low, this indicates a possible issue
 *  (loss of bus power or too high temperature) which could trigger the host controller to save critical
 *  data or go to a save state.
 *
 *  RESETB− and SAVEB−pin are open−drain pins with an internal pull−up resistor to VDDD.
 */
const struct pio knx_reset = LPC_GPIO_0_6;
const struct pio knx_save = LPC_GPIO_0_7;

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;

#define ADC_EXT1  LPC_ADC(1)
#define ADC_EXT2  LPC_ADC(2)



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
	uprintf(UART0, name);
	while (1);
}

void send_for_echo(uint8_t c)
{
	struct lpc_uart* uart1 = LPC_UART_1;
	uart1->func.buffer = c;
	status_led(red_on);
}
void got_echo(uint8_t c)
{
	struct lpc_uart* uart0 = LPC_UART_0;
	uart0->func.buffer = (c + 1);
	status_led(green_on);
}


/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, send_for_echo);
	uart_on(UART1, 38400, got_echo);


	while (1) {
	}
	return 0;
}

