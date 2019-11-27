/****************************************************************************
 *   apps/dev/ledstrip/main.c
 *
 * WS2812 Chainable leds example using Adafruit les strip
 *
 * Copyright 2013-2015 Nathael Pajani <nathael.pajani@ed3l.fr>
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
#include "drivers/adc.h"
#include "extdrv/status_led.h"

#include "extdrv/ws2812.h"

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
	/* ADC */
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	{ LPC_ADC_AD3_PIO_1_1,  LPC_IO_ANALOG },
	/* GPIO */
	{ LPC_GPIO_0_19, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio button = LPC_GPIO_0_12; /* ISP button */
const struct pio ws2812_data_out_pin = LPC_GPIO_0_19; /* Led control data pin */


/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
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


enum all_modes {
	adc_colors = 0,
	noise,
	random_colors,
	single_color_grade_red,
	single_color_grade_green,
	single_color_grade_blue,
	color_grade_red_from_adc,
	color_grade_green_from_adc,
	color_grade_blue_from_adc,
	full_grade,
	moving_dot_adc_color,
	serial_controlled,
	white,
};
static volatile uint8_t new_mode = 0;
void button_request(uint32_t gpio) {
    new_mode++;
}


/* This mode reads values from ADC[0:2] every 150ms and uses the values to set the leds.
 * Pixels are updated when all pixel are set from ADC input.
 */
void mode_adc_colors(void)
{
	static uint8_t pixel = 0;
	uint16_t red = 0, green = 0, blue = 0;

	/* Get ADC values */
	adc_get_value(&red, LPC_ADC(0));
	adc_get_value(&green, LPC_ADC(1));
	adc_get_value(&blue, LPC_ADC(2));
	/* Set one pixel */
	ws2812_set_pixel(pixel++, ((red >> 2) & 0xFF), ((green >> 2) & 0xFF), ((blue >> 2) & 0xFF));
	/* Give some time for the ADC value to change (potentiometers should be connected to ADC inputs) */
	msleep(150);
	/* give some feedback that something is going on */
	status_led(green_toggle);
	/* Buffer full, send it ! */
	if (pixel >= NB_LEDS) {
		ws2812_send_frame(0);
		pixel = 0;
	}
}

/* The "noise" mode sets the color of the led strip according to the ambient sound level.
 * It uses a Loudness sensor as input.
 * When there's no sound, the leds are Off. They turn to green with low sound level, then yellow,
 *   orange and finally red as the sound level increases.
 */
void mode_noise(void)
{
	int i = 0;
	uint16_t loudness = 0;
	uint16_t zoom = 0;
	uint8_t red = 0, green = 0, blue = 0;
	/* Get ADC value for noise */
	adc_get_value(&zoom, LPC_ADC(0));
	adc_get_value(&loudness, LPC_ADC(3));
	/* Noise is usually in the 120 - 550 range, offset and zoom on this range */
	if (loudness < 100) {
		loudness = 0;
	} else {
		zoom = ((zoom & 0x3FF) >> 7) + 1;
		loudness = ((loudness - 100) * zoom);
	}

	if (loudness < 150) {
		blue = 150 - loudness;
		green = loudness;
	} else if (loudness <= 255) {
		green = loudness;
	} else if (loudness <= 511) {
		green = 255;
		red = loudness - 256;
	} else if (loudness <= 767) {
		green = 255 - (loudness - 512);
		red = 255;
	} else {
		red = 255;
	}
	uprintf(UART0, "Loudness: %d - 0x%02x, r: %d, g:%d, b:%d, z:%d\n", loudness, loudness, red, green, blue, zoom);
	for (i = 0; i < NB_LEDS; i++) {
		ws2812_set_pixel(i, red, green, blue);
	}
	ws2812_send_frame(0);
	msleep(500); /* FIXME : Set to 100 and remove uprintf */
}

void strip_control(uint8_t c)
{
}

/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, strip_control);
	set_gpio_callback(button_request, &button, EDGE_RISING);
	status_led(none);

	/* ADC for potentiometer color settings */
	adc_on(NULL);
	adc_start_burst_conversion(ADC_MCH(0) | ADC_MCH(1) | ADC_MCH(2) | ADC_MCH(3), LPC_ADC_SEQ(0));

	/* Led strip configuration */
	ws2812_config(&ws2812_data_out_pin);

	while (1) {
		switch (new_mode) {
			case adc_colors:
				mode_adc_colors();
				break;
			case noise:
				mode_noise();
				break;
			default:
				ws2812_stop();
				msleep(1500);
				new_mode = 0;
		}
	}
	return 0;
}




