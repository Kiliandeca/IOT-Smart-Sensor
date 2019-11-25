/****************************************************************************
 *   apps/base/lcd_char/main.c
 *
 * LCD Character Display example
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
#include "drivers/timers.h"
#include "extdrv/status_led.h"
#include "drivers/ssp.h"

#include "extdrv/lcd_char.h"
#include "extdrv/max31855_thermocouple.h"

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
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio button = LPC_GPIO_0_12; /* ISP button */


/* Thermocouple reading */
const struct max31855_sensor_config thermo = {
	.ssp_bus_num = 0,
	.chip_select = LPC_GPIO_0_15,
};

/*
 * LCD Character display configuration
 */
#define TEXT_LENGTH_MAX_LCD_ONE 20
struct lcdc_definition lcd_one = {
	/* Control pins */
	.pin_reg_sel = LPC_GPIO_0_19,
	.pin_rw_sel = LPC_GPIO_0_20,
	.pin_enable = LPC_GPIO_0_21,
	/* Data pins */
	.data_pins = {
		LPC_GPIO_0_22,
		LPC_GPIO_0_23,
		LPC_GPIO_0_24,
		LPC_GPIO_0_25,
		LPC_GPIO_0_26,
		LPC_GPIO_0_27,
		LPC_GPIO_0_28,
		LPC_GPIO_0_29,
	},
	.nb_lines = 4,
	.bytes_per_line = TEXT_LENGTH_MAX_LCD_ONE,
	.eight_bits_mode = LCDC_DATA_IS_EIGTH_BITS,
	.font_size = LCDC_FONT_EIGHT_DOTS,
	.increment_type = LCDC_MODE_INCREMENT,
	.auto_display_shift = 0,
};
#define TEXT_LENGTH_MAX_LCD_TWO 16
struct lcdc_definition lcd_two = {
	/* Control pins */
	.pin_reg_sel = LPC_GPIO_0_3,
	.pin_rw_sel = LPC_GPIO_0_4,
	.pin_enable = LPC_GPIO_0_5,
	/* Data pins */
	.data_pins = {
		LPC_GPIO_0_6,
		LPC_GPIO_0_7,
		LPC_GPIO_0_8,
		LPC_GPIO_0_9,
	},
	.nb_lines = 2,
	.bytes_per_line = TEXT_LENGTH_MAX_LCD_TWO,
	.eight_bits_mode = LCDC_DATA_IS_FOUR_BITS,
	.font_size = LCDC_FONT_EIGHT_DOTS,
	.increment_type = LCDC_MODE_INCREMENT,
	.auto_display_shift = 0,
};


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

static volatile uint8_t clear_request = 0;
void button_request(uint32_t gpio) {
    clear_request = 1;
}


static volatile uint8_t display_text_buff[TEXT_LENGTH_MAX_LCD_ONE];
static volatile uint8_t update_text = 0;
void recv_text(uint8_t c)
{
	static uint8_t idx = 0;
	static uint8_t rx_buff[TEXT_LENGTH_MAX_LCD_ONE];
	rx_buff[idx++] = c;
	if ((c == '\0') || (c == '\r') || (c == '\n')) {
		update_text = 1;
		rx_buff[(idx - 1)] = ' ';
		memcpy((void*)display_text_buff, rx_buff, idx);
		idx = 0;
	}
	if (idx >= TEXT_LENGTH_MAX_LCD_ONE) {
		idx = 0;
	}
}


#define TEMP_CYCLE_DELAY 1000
/***************************************************************************** */
int main(void)
{
	int get_temp = 0;

	system_init();
	uart_on(UART0, 115200, recv_text);
	ssp_master_on(thermo.ssp_bus_num, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);
	set_gpio_callback(button_request, &button, EDGE_RISING);
	status_led(none);

	/* Thermocouple configuration */
	max31855_sensor_config(&thermo);
	uprintf(UART0, "Thermocouple config OK\n");

	/* LCD Character display configuration */
	lcdc_config(&lcd_one);
	lcdc_init(&lcd_one);
	lcdc_config(&lcd_two);
	lcdc_init(&lcd_two);
	uprintf(UART0, "Displays Init OK\n");

	/* Turn displays on */
	lcdc_on_off(&lcd_one, 1, 0, 0); /* ON, Cursor underline off, Cursor blink off */
	lcdc_on_off(&lcd_two, 1, 0, 1); /* ON, Cursor underline off, Cursor blink on */
	uprintf(UART0, "Displays ON\n");

	lcdc_move_cursor_home(&lcd_one);
	lcdc_move_cursor_home(&lcd_two);
	uprintf(UART0, "Cursors at home\n");

	lcdc_send_data_line(&lcd_one, 0, (uint8_t*)"Techno-Innov", 12);
	lcdc_send_data_line(&lcd_two, 1, (uint8_t*)"Techno-Innov", 12);
	memset((void*)display_text_buff, ' ', TEXT_LENGTH_MAX_LCD_ONE);

	while (1) {
		if (clear_request == 1) {
			/* Only clear LCD "one" */
			lcdc_clear(&lcd_one);
			lcdc_move_cursor_home(&lcd_one);
			clear_request = 0;
		}
		if (update_text == 1) {
			static int line_lcd_one = 1;
			static int line_lcd_two = 1;
			if (display_text_buff[0] != '/') {
				lcdc_send_data_line(&lcd_one, line_lcd_one, (uint8_t*)display_text_buff, lcd_one.bytes_per_line);
				memset((void*)display_text_buff, ' ', TEXT_LENGTH_MAX_LCD_ONE);
				line_lcd_one++;
				if (line_lcd_one >= lcd_one.nb_lines) {
					line_lcd_one = 0;
				}
			} else {
				lcdc_send_data_line(&lcd_two, line_lcd_two, (uint8_t*)display_text_buff, lcd_two.bytes_per_line);
				memset((void*)display_text_buff, ' ', TEXT_LENGTH_MAX_LCD_ONE);
				line_lcd_two++;
				if (line_lcd_two >= lcd_two.nb_lines) {
					line_lcd_two = 1; /* Keep first line for temperature */
				}
			}
			update_text = 0;
		}
		if (get_temp++ == TEMP_CYCLE_DELAY) {
			int centi_degrees = 0, ret = 0;
			ret = max31855_sensor_read(&thermo, NULL, &centi_degrees);
			if (ret != 0) {
				lcdc_send_data_line(&lcd_two, 0, (uint8_t*)"Temp err", 8);
			} else {
				char buff[10];
				int abs_centi = centi_degrees;
				int len = 0;
				if (centi_degrees < 0) {
					abs_centi = -centi_degrees;
				}
				len = snprintf(buff, 10, "% 4d.%02d ", (centi_degrees / 100), (abs_centi % 100));
				buff[len] = ' ';
				lcdc_send_data_line(&lcd_two, 0, (uint8_t*)buff, 8);
				lcdc_write(&lcd_two, 0xDF);
				lcdc_write(&lcd_two, 'C');
			}
			get_temp = 0;
		} else {
			msleep(1);
		}
	}
	return 0;
}




