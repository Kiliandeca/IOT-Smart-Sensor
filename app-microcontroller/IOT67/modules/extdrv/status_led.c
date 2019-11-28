/****************************************************************************
 *  extdrv/status_led.c
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
/*                Status Led                                                   */
/***************************************************************************** */


#include "core/system.h"
#include "core/pio.h"
#include "drivers/gpio.h"
#include "extdrv/status_led.h"


/***************************************************************************** */
/* Status LED is the bicolors red/green led on the GPIO Demo module */

/* This code configures the status led and enables it's use as a debug helper
 * or visual watchdog.
 * Calls to chenillard() with a value of about 25ms or more will trigger a
 * defined sequence to be displayed by the status led.
 * This sequence implies a big delay in your code (eleven times the value used
 * as argument in ms.
 *
 * If you want to use this led in a more application friendly way (with no sleep)
 * you must use a timer to call a "stepping" routine (either systick timer or any
 * of the other four timers).
 */

/* The status LED is on GPIO Port 1, pin 4 (PIO1_4) and Port 1, pin 5 (PIO1_5) */

struct pio red_led = LPC_GPIO_1_5;
struct pio green_led = LPC_GPIO_1_4;
#define LED_RED    (1 << red_led.pin)
#define LED_GREEN  (1 << green_led.pin)

void status_led_config(const struct pio* green, const struct pio* red)
{
	uint32_t mode = LPC_IO_MODE_PULL_UP | LPC_IO_DRIVE_HIGHCURENT;

	/* We must have GPIO on for status led. Calling it many times in only a waste
	 *  of time, no other side effects */
	gpio_on();
	/* Copy status led info */
	pio_copy(&red_led, red);
	pio_copy(&green_led, green);
	/* Status Led GPIO. Turn Green on. */
	config_gpio(&green_led, mode, GPIO_DIR_OUT, 1);
	config_gpio(&red_led, mode, GPIO_DIR_OUT, 0);
}

void status_led(uint32_t val)
{
	struct lpc_gpio* gpio_red = LPC_GPIO_REGS(red_led.port);
	struct lpc_gpio* gpio_green = LPC_GPIO_REGS(green_led.port);

	switch (val) {
		case red_only:
			gpio_red->set = LED_RED;
			gpio_green->clear = LED_GREEN;
			break;
		case red_on:
			gpio_red->set = LED_RED;
			break;
		case red_off:
			gpio_red->clear = LED_RED;
			break;
		case red_toggle:
			gpio_red->toggle = LED_RED;
			break;
		case green_only:
			gpio_green->set = LED_GREEN;
			gpio_red->clear = LED_RED;
			break;
		case green_on:
			gpio_green->set = LED_GREEN;
			break;
		case green_off:
			gpio_green->clear = LED_GREEN;
			break;
		case green_toggle:
			gpio_green->toggle = LED_GREEN;
			break;
		case both:
			gpio_red->set = LED_RED;
			gpio_green->set = LED_GREEN;
			break;
		case toggle:
			gpio_red->toggle = LED_RED;
			gpio_green->toggle = LED_GREEN;
			break;
		case none:
		default:
			gpio_red->clear = LED_RED;
			gpio_green->clear = LED_GREEN;
			break;
	}
}

static enum led_status steps[] = {
	red_only,
	green_only,
	none,
	both,
	none,
	red_only,
	red_only,
	none,
	green_only,
	green_only,
	none
};

void chenillard(uint32_t ms)
{
	static int n;
	status_led(steps[n++ % (sizeof steps / sizeof *steps)]);
	msleep(ms);
}
