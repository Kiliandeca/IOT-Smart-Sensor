/****************************************************************************
 *   extdrv/epaper.h
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


#ifndef EXTDRV_EPAPER_H
#define EXTDRV_EPAPER_H

#include "lib/stdint.h"
#include "core/pio.h"
#include "drivers/timers.h"


/***************************************************************************** */
/* E-Paper */
struct epaper_definition
{
	uint16_t pixels_per_line;
	uint16_t bytes_per_line;
	uint16_t bytes_per_scan;
	uint16_t lines;
	uint8_t channel[8]; /* Selects the screen size, see Doc. No. 4P008-00, section 4. */
	uint8_t gate_source_level;
	uint8_t line_termination_required;
	uint16_t stage_time;
	uint8_t spi_num;
	uint8_t pwm_timer_num;
	struct lpc_timer_pwm_config pwm_timer_conf;
	/* Input pin */
	struct pio pin_busy;
	/* Output pins */
	struct pio pin_reset;
	struct pio pin_power_ctrl;
	struct pio pin_discharge;
	struct pio pin_border_ctrl;
	struct pio pin_spi_cs;
};

enum epaper_stages {  /* mage pixel -> Display pixel */
	epaper_compensate = 0,  /* B -> W, W -> B (Current Image) */
	epaper_white,           /* B -> N, W -> W (Current Image) */
	epaper_inverse,         /* B -> N, W -> B (New Image) */
	epaper_normal,          /* B -> B, W -> W (New Image) */
};



/*
 * Configure all gpio used for e-paper display handling
 * Also calls timer_pwm_config()
 * Keeps a pointer to the epaper_definition structure, which MUST stay available.
 */
void epaper_config(struct epaper_definition* epd);


/*
 * Turn Epaper controller On.
 * Perform both COG driver Power On and Initialization.
 *
 * Must be called before any of the display / send frame functions when the display has
 *  been previously turned off or before first call
 */
void epaper_on();

/* Turn Off Epaper controller */
void epaper_off();


/*
 * Turn the whole display white or black
 * Send 0xFF for black and 0xAA for white.
 * Note : Other values will create vertical lines (or nothing) depending on the values.
 *   refer to Pervasive display documentation for more information.
 */
void epaper_uniform_display(uint8_t value);

/*
 * Send an image (whole screen), preforming all stages:
 *   - Compensate old image,
 *   - White,
 *   - Inversed new image,
 *   - New image.
 *
 * Note : unsupported yet.
 */
void epaper_display(uint8_t* old_image_data, uint8_t* new_image_data);

/*
 * Send whole image, performing only one stage.
 * Only "epaper_normal" has been tested yet.
 */
void epaper_send_frame(uint8_t* image, uint8_t stage);

/*
 * Update a few lines of the display.
 * Lines numbering starts at 0.
 * Only one stage is performed. Sending a set of "white lines" of the same size before the
 *   new lines gives better results.
 */
void epaper_send_partial_frame(uint8_t* image, uint8_t start_line, uint8_t nb_lines, uint8_t stage);

#endif /* EXTDRV_EPAPER_H */

