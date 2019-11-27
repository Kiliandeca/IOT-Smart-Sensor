/****************************************************************************
 *   extdrv/lcd_char.h
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

/* This file holds the interface to the code used to handle LCD character displays that are
 *   compatible with the Hitachi HD44780 driver.
 * There are many of them out there, and you can usually tell them by the 16-pin interface.
 *
 * Driver inspired by Arduino LiquidCrystal library code submited by
 *   David A. Mellis, Limor Fried (http://www.ladyada.net) and Tom Igoe.
 *
 * Refer to HD44780U documentation from Hitachi for more information.
 */


#ifndef EXTDRV_LCD_CHAR_H
#define EXTDRV_LCD_CHAR_H

#include "lib/stdint.h"
#include "core/pio.h"

/* IMPORTANT NOTE :
 * ALL data pins MUST be consecutive, on the same port, and the first pin in the
 *  table MUST be the lowest numbered data pin used for this driver
 */

/***************************************************************************** */
/* LCD Character display */
struct lcdc_definition
{
	uint8_t nb_lines;
	uint8_t bytes_per_line;
	uint8_t eight_bits_mode; /* Use LCDC_DATA_IS_EIGTH_BITS or LCDC_DATA_IS_FOUR_BITS */
	uint8_t font_size; /* Use LCDC_FONT_TEN_DOTS or LCDC_FONT_EIGHT_DOTS */
	uint8_t increment_type;  /* Use LCDC_MODE_INCREMENT or LCDC_MODE_DECREMENT */
	uint8_t auto_display_shift;
	/* Output pins */
	struct pio pin_reg_sel;
	struct pio pin_enable;
	struct pio pin_rw_sel;
	/* Important note : ALL data pins MUST be consecutive, on the same port, and the first pin
	 *  in the table MUST be the lowest numbered data pin used for this driver */
	struct pio data_pins[8];
	/* These are for internal use only and will be overwritten by config function.
	 * They MUST NOT be changed by the user application */
	uint32_t mask;
	uint8_t first_pin_num;
	struct lpc_gpio* port_regs;
};



/*******************************************************************************/

/*
 * Configure all gpio used for e-paper display handling
 * Also calls timer_setup()
 * Keeps a pointer to the lcdc_definition structure, which MUST stay available.
 */
void lcdc_config(struct lcdc_definition* lcd_def);

/*
 * LCD Character display initialize.
 * After power-on, display is in this state :
 *   Display clear
 *   DL=1, 8-bit interface
 *   N=0, 1-line display
 *   F=0, 5x8 dot character font
 *   D=0, Display off
 *   C=0, Cursor off
 *   B=0, Blinking off
 *   I/D=1, Increment by 1
 *   S=0, No shift
 * After this call, it is in the configuration given by the lcdc_definition structure
 *   for the fields defined in the structure, and the following state for the others :
 *   Display clear
 *   D=0, Display off
 *   C=0, Cursor off
 *   B=0, Blinking off
 */
void lcdc_init(struct lcdc_definition* lcd);


/*******************************************************************************/
/* Instructions */

/* Turn ON/Off Display
 * This function is used to control the power state of the display and the two position cursors (blink
 *   and underline);
 * Arguments set to 1 activate the function, agrguments set to 0 turn the corresponding function OFF.
 */
void lcdc_on_off(struct lcdc_definition* lcd, uint8_t on, uint8_t cursor_underline, uint8_t cursor_blink);

/* Clear display */
void lcdc_clear(struct lcdc_definition* lcd);

/* Set increment and shift mode
 * Address increment type is either LCDC_MODE_INCREMENT or LCDC_MODE_DECREMENT. This controls the direction
 *   in which the cursor or blinking is moved (right or left) when data is sent to the display (either DDRAM
 *   or CGRAM).
 * If auto_display_shift is one (LCDC_MODE_ACC_DISP_SHIFT) then it will appear that the cursor does not move
 *   but the text does.
 */
void lcdc_set_inc_and_shift_mode(struct lcdc_definition* lcd, uint8_t increment_type, uint8_t auto_display_shift);

/* Return the cursor to the home position (top left character */
void lcdc_move_cursor_home(struct lcdc_definition* lcd);

/* Move the cursor right (dir = 1) or left (dir = 0) */
void lcdc_move_cursor(struct lcdc_definition* lcd, uint8_t dir);

/* Shift the display right (dir = 1) or left (dir = 0) */
void lcdc_shift_display(struct lcdc_definition* lcd, uint8_t dir);

/* Set the cursor position to the selected line and column
 * This is the position where the next character sent will be displayed.
 * This function (or lcdc_move_cursor_home()) must be called after a write to the Character Generator RAM
 *    (CGRAM) to switch back to writting to the display.
 */
void lcdc_set_cursor_position(struct lcdc_definition* lcd, uint8_t line, uint8_t col);

/* Return the current line and collumn position of the cursor.
 * This is useful to save the current cursor position before writing to CGRAM so it can be restored later.
 */
void lcdc_get_cursor_position(struct lcdc_definition* lcd, uint8_t* col, uint8_t* line);


/*******************************************************************************/
/* Send a character to the display, at the current cursor location */
void lcdc_write(struct lcdc_definition* lcd, const uint8_t data);

/* Read a byte from the display memory, at the current cursor location */
uint8_t lcdc_read(struct lcdc_definition* lcd);

/* Send a line to the display.
 * Line numbering starts at 0
 */
void lcdc_send_data_line(struct lcdc_definition* lcd, const uint8_t line, uint8_t* line_data, uint8_t len);


/*******************************************************************************/
/* Send custom characters to display
 * Send all the data for one character.
 * char_num is the character number, it will be shifted to generate the most significant bits
 *    of the character address.
 * charmap MUST be at least 8 bytes long. Only 8 bytes will be sent.
 */
void create_char(struct lcdc_definition* lcd, uint8_t char_num, uint8_t* charmap);


/*******************************************************************************/
#define LCDC_IS_RS_INSTRUCTION  0
#define LCDC_IS_RS_DATA         1

/* Flags and mask used when reading instruction status (RS = 0 and RW = 1) */
#define LCDC_BUSY_FLAG             (0x80)
#define LCDC_ADDRESS_COUNTER_MASK  (0x7F)

/* Function register */
#define LCDC_DATA_IS_FOUR_BITS   0
#define LCDC_DATA_IS_EIGTH_BITS  1
#define LCDC_ONE_LINE            0
#define LCDC_TWO_LINES           1
#define LCDC_FONT_TEN_DOTS       1
#define LCDC_FONT_EIGHT_DOTS     0
#define LCDC_SET_FUNC(dl, n, f)   (0x20 | (((dl) & 0x01) << 4) | (((n) & 0x01) << 3) | (((f) & 0x01) << 2))

/* Display ON/OFF */
#define LCDC_DISPLAY_OFF       (0x08)
#define LCDC_DISPLAY_ON(d, c, b)  (0x08 | (((d) & 0x01) << 2) | (((c) & 0x01) << 1) | ((b) & 0x01))
/* Clear all display */
#define LCDC_DISPLAY_CLEAR     (0x01)
/* Set cursor home */
#define LCDC_CURSOR_HOME       (0x02)
/* Entry mode set */
#define LCDC_MODE_INCREMENT      1
#define LCDC_MODE_DECREMENT      0
#define LCDC_MODE_ACC_DISP_SHIFT 1
#define LCDC_MODE(inc, ads)    (0x04 | (((inc) & 0x01) << 1) | ((ads) & 0x01))
/* Cursor and display shift control */
#define LCDC_SHIFT_CURSOR     0
#define LCDC_SHIFT_DISPLAY    1
#define LCDC_SHIFT_RIGHT      1
#define LCDC_SHIFT_LEFT       0
#define LCDC_CHIFT_CTRL(sc, rl)  (0x10 | (((sc) & 0x01) << 3) | (((rl) & 0x01) << 2))

/* Set Character Generator Ram address */
#define LCDC_SET_CGRAM_ADDR(x)   (0x40 | ((x) & 0x3F))

/* Set Data Display Ram address */
#define LCDC_SET_DDRAM_ADDR(x)   (0x80 | ((x) & 0x7F))



#endif /* EXTDRV_LCD_CHAR_H */

