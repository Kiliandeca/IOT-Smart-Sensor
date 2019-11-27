/****************************************************************************
 *   extdrv/epaper.c
 *
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


/* This file holds all the code used to handle the 2.7" epaper display from embedded
 * artists on the GPIO Demo module.
 */

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/gpio.h"
#include "drivers/timers.h"
#include "drivers/ssp.h"
#include "extdrv/epaper.h"

#include "drivers/serial.h" /* for debug */
#include "extdrv/status_led.h"

struct epaper_definition* epd = NULL;


/*
 * Configure all gpio used for e-paper display handling
 * Also calls timer_setup()
 * Keeps a pointer to the epaper_definition structure, which MUST stay available.
 */
void epaper_config(struct epaper_definition* ep_def)
{
	/* Get a pointer to the epaper definition structure */
	epd = ep_def;

	/* Reset : output */
	config_gpio(&(epd->pin_reset), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
	/* Power control : output */
	config_gpio(&(epd->pin_power_ctrl), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
	/* Discharge : output */
	config_gpio(&(epd->pin_discharge), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
	/* Border control : output */
	config_gpio(&(epd->pin_border_ctrl), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
	/* Busy : input */
	config_gpio(&(epd->pin_busy), LPC_IO_MODE_PULL_UP, GPIO_DIR_IN, 0);
	/* SPI Chip Select : output */
	config_gpio(&(epd->pin_spi_cs), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 1);

	/* PWM Timer configuration */
	timer_pwm_config(epd->pwm_timer_num, &(epd->pwm_timer_conf));
}



/*******************************************************************************/
/*
 * internal functions for SPI communication with COG driver
 */
static void epaper_spi_transfer_single_byte_wait(uint16_t data)
{
	struct lpc_gpio* gpio = LPC_GPIO_REGS(epd->pin_busy.port);

	do {} while (gpio->in & (1 << epd->pin_busy.pin));

	data = spi_transfer_single_frame(epd->spi_num, data);
}

static void epaper_spi_transfer(uint8_t* out, uint8_t* in, uint8_t size)
{
	/* Set CS Low */
	gpio_clear(epd->pin_spi_cs);
	/* Perform transfer */
	spi_transfer_multiple_frames(epd->spi_num, out, in, size, 8);
	/* Release CS */
	gpio_set(epd->pin_spi_cs);
}

static void epaper_spi_send(uint8_t reg_index, uint8_t* data, uint8_t val, int length)
{
	uint8_t buff[9] = { 0 };

	/* Send register index */
	buff[0] = 0x70;
	buff[1] = reg_index;
	epaper_spi_transfer(buff, NULL, 2);
	usleep(10);
	/* Send Data */
	buff[0] = 0x72;
	if (data != NULL) {
		memcpy(&(buff[1]), data, length);
		epaper_spi_transfer(buff, NULL, (length + 1));
	} else {
		buff[1] = val;
		epaper_spi_transfer(buff, NULL, 2);
	}
	usleep(10);
}


/*******************************************************************************/
/*
 * internal functions for COG driver power on and initialisation
 */
static void epaper_cog_power_on()
{
	/* Put GPIO in the right states */
	gpio_clear(epd->pin_border_ctrl);
	gpio_clear(epd->pin_power_ctrl);
	gpio_clear(epd->pin_discharge);
	gpio_clear(epd->pin_spi_cs);
	gpio_clear(epd->pin_reset);

	/* COG Power On procedure, see Doc No. 4P008-00 From Pervasive display */
	/* Start PWM : used to eliminate possible negative voltage at low temperature. */
	timer_start(epd->pwm_timer_num);
	msleep(5); /* Keep PWM on for >=5 ms before power up */

	/* Power up display */
	gpio_set(epd->pin_power_ctrl);
	msleep(10); /* Keep PWM on for 10 more ms */

	/* Remove reset condition and border control */
	gpio_set(epd->pin_spi_cs);
	gpio_set(epd->pin_border_ctrl);
	gpio_set(epd->pin_reset);
	msleep(5); /* >=5 ms delay */
	/* Toggle reset */
	gpio_clear(epd->pin_reset);
	msleep(5); /* >=5 ms delay */
	gpio_set(epd->pin_reset);
	msleep(5); /* >=5 ms delay */
}

static void epaper_cog_initialize()
{
	struct lpc_gpio* gpio = LPC_GPIO_REGS(epd->pin_busy.port);
	uint8_t vcom[2] = {0xD0, 0x00};

	/* Wait for ready condition */
	do {} while (gpio->in & (1 << epd->pin_busy.pin));

	/* Select Channel */
	epaper_spi_send(0x01, epd->channel, 0, 8);
	/* DC/DC Frequency setting */
	epaper_spi_send(0x06, NULL, 0xFF, 1);
	/* High Power Mode Osc setting */
	epaper_spi_send(0x07, NULL, 0x9D, 1);
	/* Disable ADC */
	epaper_spi_send(0x08, NULL, 0x00, 1);
	/* Set Vcom level */
	epaper_spi_send(0x09, vcom, 0, 2);
	/* Set Gate and Source Voltage level */
	epaper_spi_send(0x04, NULL, epd->gate_source_level, 1);

	msleep(5); /* >=5 ms delay */

	/* Turn Driver Latch ON and then OFF */
	epaper_spi_send(0x03, NULL, 0x01, 1);
	epaper_spi_send(0x03, NULL, 0x00, 1);

	/* Turn on chargepump for positive voltages VGH and VDH */
	epaper_spi_send(0x05, NULL, 0x01, 1);
	msleep(30); /* >=30 ms delay */

	/* Turn off PWM */ 
	timer_stop(epd->pwm_timer_num);

	/* Turn on chargepump for negative voltages VGL and VDL */
	epaper_spi_send(0x05, NULL, 0x03, 1);
	msleep(30); /* >=30 ms delay */
	/* Turn on chargepump for Vcom driver */
	epaper_spi_send(0x05, NULL, 0x0F, 1);
	msleep(30); /* >=30 ms delay */

	/* Output enable */
	epaper_spi_send(0x02, NULL, 0x24, 1);
}



/*******************************************************************************/
/*
 * Turn Epaper controller On.
 * Perform both COG driver Power On and Initialization.
 *
 * Must be called before any of the display / send frame functions when the display has
 *  been previously turned off or before first call
 */
void epaper_on()
{
	epaper_cog_power_on();
	epaper_cog_initialize();
}


/*******************************************************************************/
/*
 * internal function used to send a line to the display.
 * Line must be long enougth for the display
 * Line numbering starts at 0
 */
static void epaper_send_data_line(const uint8_t line, uint8_t* line_data,
								  uint8_t fixed_data, uint8_t stage)
{
	int i = 0;
	uint8_t pixels = 0;

	/* Set chargepump voltage level to reduce voltage shift */
	epaper_spi_send(0x04, NULL, epd->gate_source_level, 1);
	
	/* Start with data index register */
	/* Set CS Low */
	gpio_clear(epd->pin_spi_cs);
	epaper_spi_transfer_single_byte_wait(0x70);
	epaper_spi_transfer_single_byte_wait(0x0A);
	/* Release CS */
	gpio_set(epd->pin_spi_cs);
	usleep(10);

	/* Set CS Low */
	gpio_clear(epd->pin_spi_cs);
	epaper_spi_transfer_single_byte_wait(0x72);
	/* Put even data bits first */
	if (line_data != NULL) {
		for (i = epd->bytes_per_line; i > 0; i--) {
			pixels = line_data[i - 1] & 0xAA;
			switch (stage) {
				case epaper_compensate:  /* B -> W, W -> B (Current Image) */
					pixels = 0xAA | ((pixels ^ 0xAA) >> 1);
					break;
				case epaper_white:       /* B -> N, W -> W (Current Image) */
					pixels = 0x55 + ((pixels ^ 0xAA) >> 1);
					break;
				case epaper_inverse:     /* B -> N, W -> B (New Image) */
					pixels = 0x55 | (pixels ^ 0xAA);
					break;
				case epaper_normal:      /* B -> B, W -> W (New Image) */
					pixels = 0xAA | (pixels >> 1);
					break;
			}
			pixels = pixels & 0xFF;
			epaper_spi_transfer_single_byte_wait(pixels);
		}
	} else {
		for (i = epd->bytes_per_line; i > 0; i--) {
			epaper_spi_transfer_single_byte_wait(fixed_data);
		}
	}

	/* Send scan line ... All set to 0 but one */
	for (i = 0; i < epd->bytes_per_scan; i++) {
		if (i == (line >> 2)) {
			epaper_spi_transfer_single_byte_wait((0xC0 >> ((line & 0x03) * 2)));
		} else {
			epaper_spi_transfer_single_byte_wait(0x00);
		}
	}

	/* And then put odd data bits */
	if (line_data != NULL) {
		for (i = 0; i < epd->bytes_per_line; i++) {
			uint16_t tmp = 0;
			pixels = line_data[i] & 0x55;
			switch(stage) {
				case epaper_compensate:  /* B -> W, W -> B (Current Image) */
					tmp = 0xaa | (pixels ^ 0x55);
					break;
				case epaper_white:       /* B -> N, W -> W (Current Image) */
					tmp = 0x55 + (pixels ^ 0x55);
					break;
				case epaper_inverse:     /* B -> N, W -> B (New Image) */
					tmp = 0x55 | ((pixels ^ 0x55) << 1);
					break;
				case epaper_normal:      /* B -> B, W -> W (New Image) */
					tmp = 0xaa | pixels;
					break;
			}
			/* Revert order */
			pixels = (((tmp & 0x03) << 6) | ((tmp & 0x0C) << 2) | ((tmp & 0x30) >> 2) | ((tmp & 0xC0) >> 6));
			epaper_spi_transfer_single_byte_wait(pixels);
		}
	} else {
		for (i = 0; i < epd->bytes_per_line; i++) {
			epaper_spi_transfer_single_byte_wait(fixed_data);
		}
	}

	/* Line termination */
	if (epd->line_termination_required) {
		epaper_spi_transfer_single_byte_wait(0x00);
	}

	/* Done with the frame, release chip select */
	gpio_set(epd->pin_spi_cs);

	/* Turn on output enable to send data from CoG driver to panel */
	epaper_spi_send(0x02, NULL, 0x2F, 1);
}



/*******************************************************************************/
/* Epaper display functions */

/*
 * Send whole image, performing only one stage.
 * Only "epaper_normal" has been tested yet.
 */
void epaper_send_frame(uint8_t* image, uint8_t stage)
{
	uint32_t start_tick = systick_get_tick_count();
	int i = 0;

	do {
		for (i = 0; i < epd->lines; i++) {
			epaper_send_data_line(i, (image + (i * epd->bytes_per_line)), 0, stage);
		}
	/* FIXME : start_tick will wrapp after 50 days */
	} while ((systick_get_tick_count() - start_tick) < epd->stage_time);
}

/*
 * Update a few lines of the display.
 * Lines numbering starts at 0.
 * Only one stage is performed. Sending a set of "white lines" of the same size before the
 *   new lines gives better results.
 */
void epaper_send_partial_frame(uint8_t* image, uint8_t start_line, uint8_t nb_lines, uint8_t stage)
{
	uint32_t start_tick = systick_get_tick_count();
	int i = 0;

	do {
		for (i = 0; i < nb_lines; i++) {
			epaper_send_data_line((start_line + i), (image + (i * epd->bytes_per_line)), 0, stage);
		}
	/* FIXME : start_tick will wrapp after 50 days */
	} while ((systick_get_tick_count() - start_tick) < epd->stage_time);
}

/*
 * Send an image (whole screen), preforming all stages:
 *   - Compensate old image,
 *   - White,
 *   - Inversed new image,
 *   - New image.
 *
 * Note : unsupported yet.
 */
void epaper_display(uint8_t* old_image_data, uint8_t* new_image_data)
{
	/* FIXME: perform all stages: Compensate old image, white, inversed new image, new image */
}

/*
 * Turn the whole display white or black
 * Send 0xFF for black and 0xAA for white.
 * Note : Other values will create vertical lines (or nothing) depending on the values.
 *   refer to Pervasive display documentation for more information.
 */
void epaper_uniform_display(uint8_t value)
{
	uint32_t start_tick = systick_get_tick_count();
	int i = 0;

	do {
		for (i = 0; i < epd->lines; i++) {
			epaper_send_data_line(i, NULL, value, 0);
		}
	/* FIXME : start_tick will wrapp after 50 days */
	} while ((systick_get_tick_count() - start_tick) < epd->stage_time);
}



/*******************************************************************************/
/* Turn Off Epaper controller */
void epaper_off()
{
	int i = 0;

	/* Write a Nothing frame */
	for (i = 0; i < epd->lines; i++) {
		epaper_send_data_line(i, NULL, 0x55, 0); /* Will send all 'N' */
	}
	/* Clear register data before power off by writting a dummy line
	 * A line of 0xFF is beyond the number of lines, thus no scan line will be set */
	epaper_send_data_line(0xFF, NULL, 0x55, 0);
	msleep(30); /* >25ms */
	
	/* Toggle border control */
	gpio_clear(epd->pin_border_ctrl);
	msleep(300); /* 200 - 300 ms */
	gpio_set(epd->pin_border_ctrl);

	/* Latch reset on */
	epaper_spi_send(0x03, NULL, 0x01, 1);
	/* Turn off Output enable */
	epaper_spi_send(0x02, NULL, 0x05, 1);
	/* Power off Vcom chargepump */
	epaper_spi_send(0x05, NULL, 0x0E, 1);
	/* Power off negative voltage chargepump */
	epaper_spi_send(0x05, NULL, 0x02, 1);

	/* Discharge part 1 */
	epaper_spi_send(0x04, NULL, 0x0C, 1);
	msleep(150); /* >=120ms */
	/* Power off all chargepump */
	epaper_spi_send(0x05, NULL, 0x00, 1);

	/* Turn off Oscilator */
	epaper_spi_send(0x07, NULL, 0x0D, 1);

	/* Discharge part 2 */
	/* internal */
	epaper_spi_send(0x04, NULL, 0x50, 1);
	msleep(50); /* >=40ms */
	epaper_spi_send(0x04, NULL, 0xA0, 1);
	msleep(50); /* >=40ms */
	epaper_spi_send(0x04, NULL, 0x00, 1);

	/* Turn everything to 0 */
	gpio_clear(epd->pin_border_ctrl);
	gpio_clear(epd->pin_reset);
	gpio_clear(epd->pin_power_ctrl);

	/* Discharge part 3 */
	gpio_set(epd->pin_discharge);
	msleep(180); /* >150ms */
	gpio_clear(epd->pin_discharge);

}


