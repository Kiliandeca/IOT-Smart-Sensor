/****************************************************************************
 *   apps/base/ultrasonic_sensor/main.c
 *
 * Ultrasonic range measurement example
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

/* Support for ultrasonic range measurement module.
 * Refer to readme file for further information.
 */

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/status_led.h"


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
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio sensor = LPC_GPIO_0_6; /* Ultrasonic sensor signal */


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


/* Note that clock cycles counter wraps every 89 seconds with system clock running at 48 MHz */
static volatile uint32_t pulse_start = 0;  /* Clock cycles counter upon echo start */
static volatile uint32_t pulse_end = 0;    /* Clock cycles counter upon echo end */
static volatile uint32_t pulse_duration = 0;
void pulse_feedback(uint32_t gpio) {
	static uint32_t pulse_state = 0;
	if (pulse_state == 0) {
		pulse_start = systick_get_clock_cycles();
		pulse_state = 1;
	} else {
		pulse_end = systick_get_clock_cycles();
		if (pulse_end > pulse_start) {
			pulse_duration = (pulse_end - pulse_start);
		} else {
			pulse_duration = (0xFFFFFFFF - pulse_start);
			pulse_duration += pulse_end;
		}
		pulse_state = 0;
	}
}

/* Delay between measures should be at least 50ms  */
#define DELAY 50

/***************************************************************************** */
int main(void)
{
	uint32_t next_time = 0;
	uint32_t delay = 0;

	system_init();
	uart_on(UART0, 115200, NULL);
	next_time = systick_get_tick_count();

	/* Callback on pulse start and end */
	set_gpio_callback(pulse_feedback, &sensor, EDGES_BOTH);

	uprintf(UART0, "Ultrasonic distance sensor using GPIO %d.%d\n", sensor.port, sensor.pin);
	
	while (1) {
		uint32_t distance = 0;

		/* Initiate distance mesurement */
		gpio_dir_out(sensor);
		gpio_clear(sensor);
		usleep(10);
		gpio_set(sensor);
		usleep(10);
		gpio_clear(sensor);
		gpio_dir_in(sensor);
		pulse_duration = 0;

		/* Wait for value to be available */
		while (pulse_duration == 0) {
			msleep(1);
		}
		/* Convert pulse width in us to distance in mm */
		distance = ((pulse_duration * 10) / (get_main_clock() / (1000*1000)));
		distance = distance / 29;
		/* Send value on serial */
		uprintf(UART0, "dist: %dmm\n", distance);
	
		/* And wait at least 50ms between loops */
		delay = next_time - systick_get_tick_count();
		if (delay > DELAY) {
			delay = DELAY;
		}
		msleep(delay);
		next_time += DELAY;
	}
	return 0;
}



