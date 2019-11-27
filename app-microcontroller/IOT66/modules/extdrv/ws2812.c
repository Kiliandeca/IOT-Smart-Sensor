/****************************************************************************
 *   extdrv/ws2812.c
 *
 *
 * Copyright 2013 Nathael Pajani <nathael.pajani@ed3l.fr>
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
 * Support for the WS2812 and WS2812B Chainable RGB Leds
 *
 * WS2812 protocol can be found here : https://www.adafruit.com/datasheets/WS2812.pdf
 *
 */

#include "core/system.h"
#include "core/pio.h"
#include "drivers/gpio.h"
#include "lib/string.h"
#include "extdrv/ws2812.h"


static struct pio ws2812_gpio = LPC_GPIO_0_0;

/* Configure Selected GPIO as output. */
void ws2812_config(const struct pio* gpio)
{
	config_pio(gpio, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL));
	pio_copy(&ws2812_gpio, gpio);
	gpio_dir_out(ws2812_gpio);
}

static uint8_t led_data[NB_LEDS * 3];
static uint16_t max_led = 0;
static uint32_t nb_bytes = 0;


static void ws2812_bit_sender(void)
{
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(ws2812_gpio.port);
	uint32_t gpio_bit = (1 << ws2812_gpio.pin);
	uint32_t byte_idx = 0;
	uint8_t bit_idx = 7;
	uint8_t bit = 0;

	lpc_disable_irq();

	/* Send data */
	while (byte_idx < nb_bytes) {
		bit = (led_data[byte_idx] & (0x01 << bit_idx));

		if (bit == 0) {
			/* "high" time : 350ns */
			gpio_port->set = gpio_bit;
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			/* "low" time : 800ns */
			gpio_port->clear = gpio_bit;
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
		} else {
			/* "high" time : 700ns */
			gpio_port->set = gpio_bit;
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			nop();
			/* "high" time : 600ns */
			gpio_port->clear = gpio_bit;
			nop();
			nop();
		}

		/* Move to the next bit */
		if (bit_idx == 0) {
			bit_idx = 7;
			byte_idx++;
		} else {
			bit_idx--;
		}
	}

	lpc_enable_irq();
}

/* Send led data from internal buffer (set leds to the selected color).
 * If nb_leds is 0 then all led data set using ws2812_set_pixel() since the last call
 *   to ws2812_clear_buffer(), ws2812_clear() or ws2812_stop() will be sent.
 * Call to this function will disable interrupts due to timming restrictions during
 *   the call to ws2812_bit_sender().
 */
int ws2812_send_frame(uint16_t nb_leds)
{
	if (nb_leds > NB_LEDS) {
		return -1;
	}
	if (nb_leds == 0) {
		nb_leds = max_led;
	}
	nb_bytes = (nb_leds + 1) * 3;
	ws2812_bit_sender();
	/* Reset delay */
	usleep(60);
	return 0;
}

/* Set a pixel (led) color in the data buffer */
int ws2812_set_pixel(uint16_t pixel_num, uint8_t red, uint8_t green, uint8_t blue)
{
	if (pixel_num >= NB_LEDS) {
		return -1;
	}
	led_data[ ((pixel_num * 3) + 0) ] = green;
	led_data[ ((pixel_num * 3) + 1) ] = red;
	led_data[ ((pixel_num * 3) + 2) ] = blue;
	if (max_led < pixel_num) {
		max_led = pixel_num;
	}
	return 0;
}
/* Get a pixel (led) color from the data buffer */
int ws2812_get_pixel(uint16_t pixel_num, uint8_t* red, uint8_t* green, uint8_t* blue)
{
	if (pixel_num >= NB_LEDS) {
		return -1;
	}
	*green = led_data[ ((pixel_num * 3) + 0) ];
	*red = led_data[ ((pixel_num * 3) + 1) ];
	*blue = led_data[ ((pixel_num * 3) + 2) ];
	return 0;
}

/* Clear the internal data buffer. */
void ws2812_clear_buffer(void)
{
	memset(led_data, 0, (NB_LEDS * 3));
	max_led = 0;
}

/* Clear the internal data buffer and send it to the Leds, turning them all off */
void ws2812_clear(void)
{
	/* Start at first led and send all leds off */
	ws2812_clear_buffer();
	ws2812_send_frame(NB_LEDS);
	max_led = 0;
}

void ws2812_stop(void) __attribute__ ((alias ("ws2812_clear")));

