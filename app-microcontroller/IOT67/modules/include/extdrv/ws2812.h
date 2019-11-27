/****************************************************************************
 *   extdrv/ws2812.h
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
 * Support for the WS2812 Chainable RGB Leds
 *
 * WS2812 protocol can be found here : https://www.adafruit.com/datasheets/WS2812.pdf
 *
 */

/*
 * Preliminary notice :
 * This driver will only function with a clock frequency of 48MHz (or close to) due to
 *   the use of nop() to get the right timmings.
 *
 * Internal data buffer :
 * This driver uses an internal buffer for NB_LEDS leds or "pixels". The buffer
 *   size is (3 * NB_LEDS) bytes.
 * The buffer is set pixel per pixel using the ws2812_set_pixel() function.
 * The buffer is then sent to the led strip using ws2812_send_frame().
 *
 * Driving several led strips :
 * It is possible to drive several led strips using this driver by calling the config
 *   function ws2812_config() between each ws2812_send_frame() call, and setting the led
 *   data again using the ws2812_set_pixel() function for each new led.
 * This solution is not the most adapted for several led strips, and the driver should be
 *   modified to use an external buffer which address is either passed to each function or
 *   set using a modified version of the ws2812_config() function.
 *   In this case, the timmings should be checked and updated as access to the data may
 *   require more instructions.
 *
 * Note : ws2812_send_frame() will call lpc_disable_irq() to disable all interrupts
 *   when entering the timming critical section.
 *   Use of timers and interrupts have been tried but timmings are too short even
 *   whith the micro-controller running at 48MHz and direct access to the timer and
 *   GPIO registers.
 *   Instead the function uses a few nop() to get the right timmings.
 *   
 */

#include "lib/stdint.h"
#include "core/pio.h"


/* Size of the internal buffer.
 * Change the value to the number of leds of your led strip.
 */
#define NB_LEDS  60


/* Configure the pin for the led data signal. */
void ws2812_config(const struct pio* gpio);

/* Send led data from internal buffer using the configured GPIO pin. (Set leds to the
 *   selected color).
 * If no pin have been configured, GPIO_0_0 will be used.
 * If nb_leds is 0 then all led data set using ws2812_set_pixel() since the last call
 *   to ws2812_clear_buffer(), ws2812_clear() or ws2812_stop() will be sent.
 * Call to this function will disable interrupts due to timming restrictions.
 * Return -1 on error (nb_leds above NB_LEDS), or 0 on success.
 */
int ws2812_send_frame(uint16_t nb_leds);

/* Set a pixel (led) color in the data buffer (frame)
 * The pixel number 'pixel_num' is the led offset in the led strip.
 * 'red', 'green' and 'blue' are the color values of the pixel. A value of 0 is off,
 *   while a value of 0xFF (255) is full brightness.
 * Return -1 on error (pixel_num above NB_LEDS), or 0 on success.
 */
int ws2812_set_pixel(uint16_t pixel_num, uint8_t red, uint8_t green, uint8_t blue);

/* Get a pixel (led) color from the data buffer */
int ws2812_get_pixel(uint16_t pixel_num, uint8_t* red, uint8_t* green, uint8_t* blue);

/* Clear the internal data buffer. */
void ws2812_clear_buffer(void);

/* Clear the internal data buffer and send it to the Leds, turning them all off */
void ws2812_clear(void);
/*	Alias for ws2812_clear */
void ws2812_stop(void);

