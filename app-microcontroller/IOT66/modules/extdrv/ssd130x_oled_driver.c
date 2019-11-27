/****************************************************************************
 *   extdrv/ssd130x_oled_driver.c
 *
 * I2C Driver for 128x64 oled display drivers
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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
#include "lib/stdint.h"
#include "lib/errno.h"
#include "lib/string.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "drivers/ssp.h"
#include "extdrv/ssd130x_oled_driver.h"


/***************************************************************************** */

/* SSD1306 starts with control reg set to 0x7F and in normal display mode,
 *  which is equivalent to 0xA4 command.
 * (Refer to chapter 8.5 of SSD1306 manual)
 *
 * The power on sequence is described in chapter 8.9 of SSD1306 manual.
 *  After VDD power stable, Reset signal should be pulled low for at least 3us
 *  and then VCC turned on.
 *  Once VCC stable, send 0xAF command to turn display ON and wait for 100ms.
 *
 * The power off sequence starts by sending 0xAE command, then turn off VCC,
 *  wait for 100ms, and turn off VDD.
 *
 * The Graphic Display Data RAM (GDDRAM) is 128 x 64 bits divided in 8 pages
 *  of 128 bytes each. One page represents 8 lines of 128 dots.
 * When writing one byte to one page, the byte fills one collumn (8 vertical
 *  pixels), and the next byte fills the next collumn, and so on.
 * (Refer to chapter 8.7 of SSD1306 manual)
 *
 * No data read is available in serial (SPI and I2C ?) mode.
 * The serial interface mode is always in write mode.
 *
 * The GDDRAM column address pointer will be increased automatically by one
 *  after each data write.
 */


 /*
 * Note : This driver does not implemment and support page adressing. All page
 *  addressing commands are not supported.
 * This affects the following commands :
 *   - All commands 0x00 to 0x1F are not supported (set column start for page
 *       addressing mode).
 *   - ssd130x_set_mem_addressing_mode() (0x20) supports only
 *       SSD130x_ADDR_TYPE_HORIZONTAL and SSD130x_ADDR_TYPE_VERTICAL
 *   - Set page start address (0xB0 to 0xB7)
 *
 * Note some other commands are not supported:
 *   - Set display start line (0x40 to 0x7F)
 *   - Set segment remap (0xA0 and 0xA1)
 *   - Set multiplex ratio (0xA8)
 *   - Set display clock divide ration and oscilator frequency (0xD5)
 *   - Set pre-charge period (0xD9)
 *   - Set COM pins hardware configuration (0xDA)
 *   - Set Vcomh deselect level (0xDB)
 *   - No OP (0xE3)
 */

/***************************************************************************** */
/* Commands */
#define CMD_BUF_SIZE 24
int ssd130x_send_command(struct oled_display* conf, uint8_t cmd, uint8_t* data, uint8_t len)
{
	int i;

	if (conf->bus_type == SSD130x_BUS_SPI) {
		gpio_clear(conf->gpio_dc);
		gpio_clear(conf->gpio_cs);
		spi_transfer_single_frame(conf->bus_num, cmd);
		for (i = 0; i < len; i++) {
			spi_transfer_single_frame(conf->bus_num, data[i]);
		}
		gpio_set(conf->gpio_cs);
	} else if (conf->bus_type == SSD130x_BUS_I2C) {
		char cmd_buf[CMD_BUF_SIZE] = { conf->address, SSD130x_NEXT_BYTE_CMD, cmd, };
		int ret;

		if (2 * len > (CMD_BUF_SIZE - 3)) {
			return -EINVAL;
		}

		if (len != 0) {
			for (i = 0; i < len; i++) {
				cmd_buf[ 3 + (2 * i) ] = SSD130x_NEXT_BYTE_CMD;
				cmd_buf[ 4 + (2 * i) ] = data[i];
			}
		}

		do {
			ret = i2c_write(conf->bus_num, cmd_buf, (3 + (len * 2)), NULL);
		} while (ret == -EAGAIN);
		if (ret != (3 + (len * 2))) {
			conf->probe_ok = 0;
			return ret;
		}
	} else {
		return -EPROTO;
	}
	return 0;
}


/* Set memory adressing mode (0x20)
 * There are 3 different memory addressing mode in SSD130x: page addressing
 *  mode, horizontal addressing mode and vertical addressing mode.
 * This command sets the way of memory addressing into one of the above three
 *   modes.
 * (Refer to chapter 10.1.3 of SSD1306 manual)
 *
 * Note : This driver does not implemment and support page adressing. All page
 *  addressing commands are not supported.
 */
int ssd130x_set_mem_addressing_mode(struct oled_display* conf, uint8_t type)
{
	return ssd130x_send_command(conf, SSD130x_CMD_ADDR_TYPE, &type, 1);
}

/* Set column address (0x21)
 * This triple byte command specifies column start address and end address of
 *  the display data RAM.
 * This command also sets the column address pointer to column start address.
 * This pointer is used to define the current read/write column address in
 *  graphic display data RAM.
 * If horizontal address increment mode is enabled by command 20h, after
 *  finishing read/write one column data, it is incremented automatically to
 *  the next column address.
 * Whenever the column address pointer finishes accessing the end column
 *  address, it is reset back to start column address and the row address is
 *  incremented to the next row.
 */
int ssd130x_set_column_address(struct oled_display* conf, uint8_t col_start, uint8_t col_end)
{
	uint8_t buf[2] = { col_start, col_end };
	return ssd130x_send_command(conf, SSD130x_CMD_COL_ADDR, buf, 2);
}

/* Set page address (0x22)
 * This triple byte command specifies page start address and end address of the
 *  display data RAM.
 * This command also sets the page address pointer to page start address.
 * This pointer is used to define the current read/write page address in
 *  graphic display data RAM.
 * If vertical address increment mode is enabled by command 20h, after
 *  finishing read/write one page data, it is incremented automatically to the
 *  next page address.
 * Whenever the page address pointer finishes accessing the end page address,
 *  it is reset back to start page address.
 */
int ssd130x_set_page_address(struct oled_display* conf, uint8_t page_start, uint8_t page_end)
{
	uint8_t buf[2] = { page_start, page_end, };
	return ssd130x_send_command(conf, SSD130x_CMD_PAGE_ADDR, buf, 2);
}

/* Set contrast (0x81)
 * The contrast value is between 0 and 256
 */
int ssd130x_set_contrast(struct oled_display* conf)
{
	return ssd130x_send_command(conf, SSD130x_CMD_CONTRAST, &(conf->contrast), 1);
}

/* Set display ON from GDDRAM (0xA4) or regardless of GDDRAM (0xA5)
 * use_ram is either SSD130x_DISP_RAM or SSD130x_DISP_BLANK
 */
int ssd130x_set_display_on(struct oled_display* conf, uint8_t use_ram)
{
	if (use_ram == SSD130x_DISP_BLANK) {
		return ssd130x_send_command(conf, SSD130x_CMD_DISP_NORAM, NULL, 0);
	} else {
		return ssd130x_send_command(conf, SSD130x_CMD_DISP_RAM, NULL, 0);
	}
}

/* Set display to normal or reverse video
 * status is either SSD130x_DISP_NORMAL or SSD130x_DISP_REVERSE
 */
int ssd130x_display_video_reverse(struct oled_display* conf)
{
	if (conf->video_mode == SSD130x_DISP_REVERSE) {
		return ssd130x_send_command(conf, SSD130x_CMD_DISP_REVERSE, NULL, 0);
	} else {
		return ssd130x_send_command(conf, SSD130x_CMD_DISP_NORMAL, NULL, 0);
	}
}

/* Set display ON/OFF
 * status is either SSD130x_DISP_ON or SSD130x_DISP_OFF
 */
int ssd130x_display_power(struct oled_display* conf, uint8_t status)
{
	if (status == SSD130x_DISP_OFF) {
		return ssd130x_send_command(conf, SSD130x_CMD_DISP_OFF, NULL, 0);
	} else {
		return ssd130x_send_command(conf, SSD130x_CMD_DISP_ON, NULL, 0);
	}
}

/* Set scan direction (0xC0 / 0xC8)
 * Scan direction is either SSD130x_SCAN_TOP_BOTTOM or SSD130x_SCAN_BOTTOM_TOP
 */
int ssd130x_set_scan_direction(struct oled_display* conf)
{
	if (conf->scan_dir == SSD130x_SCAN_TOP_BOTTOM) {
		return ssd130x_send_command(conf, SSD130x_CMD_COM_SCAN_NORMAL, NULL, 0);
	} else {
		return ssd130x_send_command(conf, SSD130x_CMD_COM_SCAN_REVERSE, NULL, 0);
	}
}
/* Set read direction (0xA0 / 0xA1)
 * Scan direction is either SSD130x_CMD_SEG0_MAP_RIGHT or SSD130x_CMD_SEG0_MAP_LEFT
 */
int ssd130x_set_read_direction(struct oled_display* conf)
{
	if (conf->read_dir == SSD130x_RIGHT_TO_LEFT) {
		return ssd130x_send_command(conf, SSD130x_CMD_SEG0_MAP_RIGHT, NULL, 0);
	} else {
		return ssd130x_send_command(conf, SSD130x_CMD_SEG0_MAP_LEFT, NULL, 0);
	}
}

/* Set display offset
 * Move the display up or down by a given number of lines
 * FIXME : Check influence of COM output scan direction.
 */
int ssd130x_set_display_offset(struct oled_display* conf, uint8_t dir, uint8_t nb_lines)
{
	uint8_t offset = 0;
	if (nb_lines >= SSD130x_NB_LINES) {
		return -EINVAL;
	}
	if (dir == SSD130x_MOVE_TOP) {
		offset = SSD130x_OFFSET_DATA(nb_lines);
	} else {
		offset = SSD130x_OFFSET_DATA(64 - nb_lines);
	}
	return ssd130x_send_command(conf, SSD130x_CMD_DISPLAY_OFFSET, &offset, 1);
}



/***************************************************************************** */
/* Init */
int ssd130x_display_off(struct oled_display* conf)
{
	return ssd130x_display_power(conf, SSD130x_DISP_OFF);
}

int ssd130x_display_on(struct oled_display* conf)
{
	int ret = 0;
	uint8_t val = 0;

	conf->fullscreen = 0;

	if (conf->bus_type == SSD130x_BUS_SPI) {
		config_gpio(&conf->gpio_cs, 0, GPIO_DIR_OUT, 1);
		config_gpio(&conf->gpio_dc, 0, GPIO_DIR_OUT, 1);

		config_gpio(&conf->gpio_rst, 0, GPIO_DIR_OUT, 0);
		usleep(5); /* at least 3us */
		gpio_set(conf->gpio_rst);
		msleep(1);
	}
	/* Adafruit OLED displays need to pull a down signal on reset */
	if (conf->bus_type == SSD130x_BUS_I2C) {
		config_gpio(&conf->gpio_rst, 0, GPIO_DIR_OUT, 1);
		msleep(1); 
		gpio_clear(conf->gpio_rst);
		msleep(10);
		gpio_set(conf->gpio_rst);
	}
	/* Display OFF */
	
	ret = ssd130x_display_power(conf, SSD130x_DISP_OFF);  /* 0xAE */
	if (ret != 0) {
		return ret;
	}

	if (conf->charge_pump == SSD130x_INTERNAL_PUMP) {
		val = SSD130x_CMD_CHARGE_INTERN; /* 0x14 */
		ssd130x_send_command(conf, SSD130x_CMD_CHARGE_PUMP, &val, 1); /* 0x8D */
	}

	ret = ssd130x_set_mem_addressing_mode(conf, SSD130x_ADDR_TYPE_HORIZONTAL);
	if (ret != 0) {
		return ret;
	}

	ret = ssd130x_set_scan_direction(conf);
	if (ret != 0) {
		return ret;
	}
	ret = ssd130x_set_read_direction(conf);
	if (ret != 0) {
		return ret;
	}

	if (conf->charge_pump == SSD130x_INTERNAL_PUMP) {
		val = 0xF1;
		ssd130x_send_command(conf, SSD130x_CMD_SET_PRECHARGE, &val, 1); /* 0xD9 */
		val = SSD130x_VCOM_083;
		ssd130x_send_command(conf, SSD130x_CMD_VCOM_LEVEL, &val, 1);  /* 0xDB */
	}

	ret = ssd130x_set_display_on(conf, SSD130x_DISP_RAM);
	if (ret != 0) {
		return ret;
	}

	return ssd130x_display_power(conf, SSD130x_DISP_ON);
}



/***************************************************************************** */
/* Data */

/* Our internal buffer for the whole display.
 * Graphical data starts at byte 4. The first four bytes are here for temporary
 *  storage of display data during I2C frame transfer.
 */
//static uint8_t gddram[ 4 + GDDRAM_SIZE ];
//static uint8_t* gddram_start = (gddram + 4);


int ssd130x_send_data(struct oled_display* conf, uint8_t* start, uint16_t len)
{
	int ret;

	if (conf->bus_type == SSD130x_BUS_SPI) {
		gpio_set(conf->gpio_dc);
		gpio_clear(conf->gpio_cs);
		ret = spi_transfer_multiple_frames(conf->bus_num, start, NULL, len, 8);
		gpio_set(conf->gpio_cs);
		if (ret != len) {
			return ret;
		}
	} else if (conf->bus_type == SSD130x_BUS_I2C) {
		int (*write)(uint8_t, const void *, size_t, const void*);

		/* Check that start and satrt + len are within buffer */

		/* Copy previous two bytes to storage area (gddram[0] and gddram[1]) */
		conf->gddram[0] = *(start - 2);
		conf->gddram[1] = *(start - 1);

		/* Setup I2C transfer */
		*(start - 2) = conf->address;
		*(start - 1) = SSD130x_DATA_ONLY;

		/* Send data on I2C bus */
		write = conf->async ? i2c_write_async : i2c_write;
		do {
			ret = write(conf->bus_num, (start - 2), (2 + len), NULL);
		} while (ret == -EAGAIN);

		/* Restore gddram data */
		*(start - 2) = conf->gddram[0];
		*(start - 1) = conf->gddram[1];

		if (ret != (2 + len)) {
			return ret;
		}
	} else {
		return -EPROTO;
	}
	return 0;
}

/* Update what is really displayed */
int ssd130x_display_full_screen(struct oled_display* conf)
{
	int ret;

	if (!conf->fullscreen) {
		ret = ssd130x_set_column_address(conf, 0, 127);
		if (ret != 0) {
			return ret;
		}
		ret = ssd130x_set_page_address(conf, 0, 7);
		if (ret != 0) {
			return ret;
		}
		conf->fullscreen = 1;
	}

	if (conf->bus_type == SSD130x_BUS_SPI) {
		ret = ssd130x_send_data(conf, conf->gddram + 4, GDDRAM_SIZE);
	} else if (conf->bus_type == SSD130x_BUS_I2C) {
		/* Setup I2C transfer */
		*(conf->gddram + 2) = conf->address;
		*(conf->gddram + 3) = SSD130x_DATA_ONLY;

		/* Send data on I2C bus */
		int (*write)(uint8_t, const void *, size_t, const void*);
		write = conf->async ? i2c_write_async : i2c_write;
		do {
			ret = write(conf->bus_num, conf->gddram + 2, 2 + GDDRAM_SIZE, NULL);
		} while (ret == -EAGAIN);
	} else {
		return -EPROTO;
	}
	return ret;
}

/* Change a "tile" in the GDDRAM memory.
 * A tile is a 8x8 pixels region, aligned on a 8x8 grid representation of the display.
 *  x0 and y0 are in number of tiles.
 */
int ssd130x_update_tile(struct oled_display* conf, uint8_t x0, uint8_t y0)
{
	uint8_t* addr = conf->gddram + 4 + (y0 * 128) + x0 * 8;
	int ret = 0;

	conf->fullscreen = 0;

	ret = ssd130x_set_column_address(conf, (x0 * 8), (((x0 + 1) * 8) - 1));
	if (ret != 0) {
		return ret;
	}
	ret = ssd130x_set_page_address(conf, y0, y0);
	if (ret != 0) {
		return ret;
	}
	ret = ssd130x_send_data(conf, addr, 8);
	if (ret != 0) {
		return ret;
	}
	return 0;
}

int ssd130x_update_modified(struct oled_display* conf)
{
	return 0;
}

