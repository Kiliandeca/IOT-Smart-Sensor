/****************************************************************************
 *   extdrv/lcd.c
 *
 *  LCD character display support
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


/* This file holds all the code used to handle LCD character displays that are compatible
 *   with the Hitachi HD44780 driver.
 * There are many of them out there, and you can usually tell them by the 16-pin interface.
 *
 * Most code and comments are inspired by Arduino LiquidCrystal library code submited by
 *   David A. Mellis, Limor Fried (http://www.ladyada.net) and Tom Igoe.
 *
 * Refer to HD44780U documentation from Hitachi for more information.
 */

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/gpio.h"
#include "drivers/timers.h"
#include "drivers/ssp.h"
#include "extdrv/lcd_char.h"

#include "drivers/serial.h" /* for debug */
#include "extdrv/status_led.h"


/*
 * Configure all gpio used for e-paper display handling
 * Also calls timer_setup()
 * Keeps a pointer to the lcdc_definition structure, which MUST stay available.
 */
void lcdc_config(struct lcdc_definition* lcd)
{
	int i = 0, nb_data_pins = 4;

	if (lcd->eight_bits_mode) {
		nb_data_pins = 8;
	}
	/* Register select pin : output and low (instructions) */
	config_gpio(&(lcd->pin_reg_sel), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
	/* Enable pin : output and low */
	config_gpio(&(lcd->pin_enable), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
	/* Raed/Write select pin : output and low (write) */
	config_gpio(&(lcd->pin_rw_sel), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
	/* Data : Input/output, set as output for initial state. */
	lcd->mask = ~(0);
	lcd->first_pin_num = lcd->data_pins[0].pin;
	lcd->port_regs = LPC_GPIO_REGS(lcd->data_pins[0].port);
	for (i = 0; i < nb_data_pins; i++) {
		config_gpio(&(lcd->data_pins[i]), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
		lcd->mask &= ~(0x01 << lcd->data_pins[i].pin);
	}
	
}


/*******************************************************************************/
/* Internal function for write data transfer
 * Having data on consecutive pins allows for the same function to be used for both 4 and 8 bits data width.
 * Masking will take care of the rest.
 */
static void lcdc_write_byte(struct lcdc_definition* lcd, uint8_t data)
{
	uint32_t oldmask = lcd->port_regs->mask;
	/* Disable interrupts : we are about to set a port mask, thus any interrupt routine trying to
	 *   access a pin on this port will be unable to perform it's job */
	lpc_disable_irq();
	/* Set data first */
	lcd->port_regs->mask = lcd->mask;
	lcd->port_regs->out = (data << lcd->first_pin_num);
	lcd->port_regs->mask = oldmask;
	/* Old mask restored, restore IRQ */
	lpc_enable_irq();
	/* And generate enable pulse: 
	 *   enable is already low
	 *   address (type + r/w) has been stable for more than 50ns (about 2.5 instructions at 48MHz, we
	 *      have a minimum of three above)
	 *   data is set, so it will be stable for more than 100ns before end of pulse
	 *   so we generate a 1us pulse which is far more than the 290ns minimum pulse width.
	 */
	usleep(1);
	gpio_set(lcd->pin_enable);
	usleep(1);
	gpio_clear(lcd->pin_enable);
	/* The cycle time must be above 650ns, already the case, but arduino code waits for 100us ... */
	usleep(100);
}

static uint8_t lcdc_busy(struct lcdc_definition* lcd);

/* This is the LCD write function.
 * It takes care of r/d and r/w selection signals and performs the call(s) to the byte write function
 *    according to the data width configuration.
 * It also waits for the busy indicator to get low before returning when issuing instructions.
 */
static void lcdc_perform_write(struct lcdc_definition* lcd, uint8_t data, uint8_t type)
{
	/* Type is Instruction (type == 0) or Data (type == 1) */
	if (type) {
		gpio_set(lcd->pin_reg_sel);
	} else {
		gpio_clear(lcd->pin_reg_sel);
	}
	/* We are writting */
	gpio_clear(lcd->pin_rw_sel);
	/* Use one or two cycles depending on bus width */
	if (lcd->eight_bits_mode) {
		lcdc_write_byte(lcd, data);
	} else {
		lcdc_write_byte(lcd, (data >> 4) & 0x0F);
		lcdc_write_byte(lcd, data & 0x0F);
	}
	/* When sending an instruction, wait for the busy bit to clear */
	if (type == LCDC_IS_RS_INSTRUCTION) {
		while (lcdc_busy(lcd)) {
			usleep(200);
		};
	}

}

/* Internal function for read data transfer
 * As for write, having data on consecutive pins allows for the same function to be used for
 *   both 4 and 8 bits data width.
 * Masking will take care of the rest.
 */
static uint8_t lcdc_read_byte(struct lcdc_definition* lcd)
{
	uint32_t oldmask = lcd->port_regs->mask;
	uint8_t data = 0;

	/* Erase data while pins are in output mode (mask has 0's for unmasked pins) */
	lcd->port_regs->clear = ~(lcd->mask);
	/* Set pins as input (mask has 0's for unmasked pins) */
	lcd->port_regs->data_dir &= lcd->mask;
	/* Generate enable pulse:
	 *   enable is already low
	 *   address (type + r/w) has been stable for more than 50ns (about 2.5 instructions at 48MHz, we
	 *      have a minimum of two above, and setting enable pin high makes for three)
	 *   generate a 1us pulse which is far more than the 290ns minimum pulse width.
	 *   data is available 150ns after enable goes high, so the 1us delay is also much more than required.
	 */
	usleep(1);
	gpio_set(lcd->pin_enable);
	usleep(1);
	/* End of Enable pulse */
	gpio_clear(lcd->pin_enable);
	/* Read data */
	/* Disable interrupts : we are about to set a port mask, thus any interrupt routine trying to
	 *   access a pin on this port will be unable to perform it's job */
	lpc_disable_irq();
	/* Set mask */
	lcd->port_regs->mask = lcd->mask;
	data = lcd->port_regs->in;
	/* Restore mask */
	lcd->port_regs->mask = oldmask;
	/* Old mask restored, restore IRQ */
	lpc_enable_irq();
	/* Restore output config for pins (mask has 0's for unmasked pins) */
	lcd->port_regs->data_dir |= ~(lcd->mask);
	/* The cycle time must be above 650ns, already the case, but arduino code waits for 100us ... */
	usleep(100);
	/* And shift the data before returning it */
	return (data >> lcd->first_pin_num);
}

/* This is the LCD read function.
 * It takes care of r/d and r/w selection signals and performs the call(s) to the byte read function
 *    according to the data width configuration.
 */
static uint8_t lcdc_perform_read(struct lcdc_definition* lcd, uint8_t type)
{
	uint8_t data = 0;
	/* Type is Instruction or Data */
	if (type) {
		gpio_set(lcd->pin_reg_sel);
	} else {
		gpio_clear(lcd->pin_reg_sel);
	}
	/* We are reading */
	gpio_set(lcd->pin_rw_sel);
	/* Use one or two cycles depending on bus width */
	if (lcd->eight_bits_mode) {
		data = lcdc_read_byte(lcd);
	} else {
		data = (lcdc_read_byte(lcd) << 4);
		data |= lcdc_read_byte(lcd);
	}
	return data;
}


/* Check the busy bit.
 * This is used to wait for the end of the instruction execution.
 */
static uint8_t lcdc_busy(struct lcdc_definition* lcd)
{
	uint8_t data = lcdc_perform_read(lcd, LCDC_IS_RS_INSTRUCTION);
	if (data & LCDC_BUSY_FLAG) {
		return 1;
	}
	return 0;
}


/*******************************************************************************/
/* Instructions */

/* Turn ON/Off Display
 * This function is used to control the power state of the display and the two position cursors (blink
 *   and underline)
 * Arguments set to 1 activate the function, agrguments set to 0 turn the corresponding function OFF.
 */
void lcdc_on_off(struct lcdc_definition* lcd, uint8_t on, uint8_t cursor_underline, uint8_t cursor_blink)
{
	lcdc_perform_write(lcd, LCDC_DISPLAY_ON(on, cursor_underline, cursor_blink), LCDC_IS_RS_INSTRUCTION);
}

/* Clear display */
void lcdc_clear(struct lcdc_definition* lcd)
{
	lcdc_perform_write(lcd, LCDC_DISPLAY_CLEAR, LCDC_IS_RS_INSTRUCTION);
	msleep(2);
}

/* Set increment and shift mode
 * Address increment type is either LCDC_MODE_INCREMENT or LCDC_MODE_DECREMENT. This controls the direction
 *   in which the cursor or blinking is moved (right or left) when data is sent to the display (either DDRAM
 *   or CGRAM).
 * If auto_display_shift is one (LCDC_MODE_ACC_DISP_SHIFT) then it will appear that the cursor does not move
 *   but the text does.
 */
void lcdc_set_inc_and_shift_mode(struct lcdc_definition* lcd, uint8_t increment_type, uint8_t auto_display_shift)
{
	lcdc_perform_write(lcd, LCDC_MODE(increment_type, auto_display_shift), LCDC_IS_RS_INSTRUCTION);
}

/* Return the cursor to the home position (top left character */
void lcdc_move_cursor_home(struct lcdc_definition* lcd)
{
	lcdc_perform_write(lcd, LCDC_CURSOR_HOME, LCDC_IS_RS_INSTRUCTION);
	msleep(2);
}

/* Move the cursor right (dir = 1) or left (dir = 0) */
void lcdc_move_cursor(struct lcdc_definition* lcd, uint8_t dir)
{
	lcdc_perform_write(lcd, LCDC_CHIFT_CTRL(LCDC_SHIFT_CURSOR, dir), LCDC_IS_RS_INSTRUCTION);
}

/* Shift the display right (dir = 1) or left (dir = 0) */
void lcdc_shift_display(struct lcdc_definition* lcd, uint8_t dir)
{
	lcdc_perform_write(lcd, LCDC_CHIFT_CTRL(LCDC_SHIFT_DISPLAY, dir), LCDC_IS_RS_INSTRUCTION);
}

/* Set the cursor position to the selected line and column
 * This is the position where the next character sent will be displayed.
 * This function (or lcdc_move_cursor_home()) must be called after a write to the Character Generator RAM
 *    (CGRAM) to switch back to writting to the display.
 */
void lcdc_set_cursor_position(struct lcdc_definition* lcd, uint8_t line, uint8_t col)
{
	int line_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (line >= lcd->nb_lines) {
		line = (lcd->nb_lines - 1);
	}
	lcdc_perform_write(lcd, LCDC_SET_DDRAM_ADDR(col + line_offsets[line]), LCDC_IS_RS_INSTRUCTION);
}


/* Return the current line and collumn position of the cursor.
 * This is useful to save the current cursor position before writing to CGRAM so it can be restored later.
 */
void lcdc_get_cursor_position(struct lcdc_definition* lcd, uint8_t* col, uint8_t* line)
{
	uint8_t data = 0;
	/* Wait for busy bit to clear (should always be the case as we wait for it after all instructions) */
	while (lcdc_busy(lcd));
	/* Sleep a (very) little bit because the address is updated after the busy bit is cleared */
	usleep(1);
	data = lcdc_perform_read(lcd, LCDC_IS_RS_INSTRUCTION);
	/* Compute line and collumn according to value of data */
	*col = (data & 0x3F);
	*line = 0;
	if (data >= 0x40) {
		(*line)++;
	}
	if (*col >= 0x14) {
		*col -= 0x14;
		*line += 2;
	}
}

/*******************************************************************************/
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
void lcdc_init(struct lcdc_definition* lcd)
{
	uint32_t cur_tick = systick_get_tick_count();
	uint8_t val = 0;

	if (!is_systick_running()) {
		systick_timer_on(1);
		systick_start();
	}
	/* We must provide an initial 50ms delay after power-up for initial reset routine of the LCD
	 * This could be as little as 15ms if we were using 5V power, but our micro-controller is 3.3V supplied
	 * Refer to HD44780 documentation pages 45 and 56 for more information on initialization instructions
	 */
	if (cur_tick < 50) {
		msleep(50 - cur_tick);
	}
	/* We will use directly the byte / half_byte instructions, so we must clear the instruction
	 *   indicator ourself (instruction mode is active when corresponding signal is low) */
	gpio_clear(lcd->pin_reg_sel);
	gpio_clear(lcd->pin_rw_sel);
	if (lcd->eight_bits_mode) {
		lcdc_write_byte(lcd, 0x30);
		msleep(5); /* Min 4.1ms */
		lcdc_write_byte(lcd, 0x30);
	} else {
		lcdc_write_byte(lcd, 0x03);
		msleep(5); /* Min 4.1ms */
		lcdc_write_byte(lcd, 0x03);
	}
	usleep(150); /* Min 100us */
	lcdc_perform_write(lcd, 0x32, LCDC_IS_RS_INSTRUCTION);
	/* Set function instruction */
	val = LCDC_SET_FUNC(lcd->eight_bits_mode, (lcd->nb_lines > 1), lcd->font_size);
	lcdc_perform_write(lcd, val, LCDC_IS_RS_INSTRUCTION);
	/* Turn off display */
	lcdc_on_off(lcd, 0, 0, 0);
	/* Clear display */
	lcdc_clear(lcd);
	/* Set mode */
	lcdc_set_inc_and_shift_mode(lcd, lcd->increment_type, lcd->auto_display_shift);
}


/*******************************************************************************/
/* Send a character to the display, at the current cursor location */
void lcdc_write(struct lcdc_definition* lcd, const uint8_t data)
{
	lcdc_perform_write(lcd, data, LCDC_IS_RS_DATA);
}

/* Read a byte from the display memory, at the current cursor location */
uint8_t lcdc_read(struct lcdc_definition* lcd)
{
	return lcdc_perform_read(lcd, LCDC_IS_RS_DATA);
}

/* Send a line to the display.
 * Line numbering starts at 0
 */
void lcdc_send_data_line(struct lcdc_definition* lcd, const uint8_t line, uint8_t* line_data, uint8_t len)
{
	int i = 0;
	/* Do not go beyond line length */
	if (len > lcd->bytes_per_line) {
		len = lcd->bytes_per_line;
	}
	/* Move the cursor at the beginning or end of line depending on the writing direction */
	if (lcd->increment_type == LCDC_MODE_INCREMENT) {
		lcdc_set_cursor_position(lcd, line, 0);
	} else {
		lcdc_set_cursor_position(lcd, line, lcd->bytes_per_line);
	}
	/* And display the data */
	for (i = 0; i < len; i++) {
		lcdc_write(lcd, line_data[i]);
	}
}



/*******************************************************************************/
/* Send custom characters to display
 * Send all the data for one character.
 * char_num is the character number, it will be shifted to generate the most significant bits
 *    of the character address.
 * charmap MUST be at least 8 bytes long. Only 8 bytes will be sent.
 */
void create_char(struct lcdc_definition* lcd, uint8_t char_num, uint8_t* charmap)
{
	int i = 0;
	if (char_num >= 8) {
		return; /* Only eight user defined characters available. */
	}
	/* A 5x8 patern uses 8 bytes for one character, so character address is shifted by 3 */
	lcdc_perform_write(lcd, LCDC_SET_CGRAM_ADDR(char_num << 3), LCDC_IS_RS_INSTRUCTION);
	/* And send the data */
	for (i = 0; i < 8; i++) {
		lcdc_perform_write(lcd, charmap[i], LCDC_IS_RS_DATA);
	}
}


