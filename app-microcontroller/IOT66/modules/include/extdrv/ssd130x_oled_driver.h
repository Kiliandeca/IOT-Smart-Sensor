/****************************************************************************
 *   extdrv/ssd130x_oled_driver.h
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

#ifndef EXTDRV_SSD130X_OLED_DRIVER_H
#define EXTDRV_SSD130X_OLED_DRIVER_H

#include "lib/stdint.h"
#include "drivers/gpio.h"


/***************************************************************************** */
/* Oled Display */
struct oled_display {
	uint8_t  bus_type; /* I2C or SPI */
	uint8_t  address; /* 8 bits address */
	uint8_t  bus_num; /* I2C/SPI bus number */
	uint8_t  probe_ok;
	uint8_t  charge_pump;
	uint8_t  video_mode;
	uint8_t  contrast;
	uint8_t  scan_dir;
	uint8_t  read_dir;
	uint8_t  display_offset_dir;
	uint8_t  display_offset;
	uint8_t* gddram;
	uint8_t  async;
	/* spi */
	struct pio gpio_dc;
	struct pio gpio_cs;
	struct pio gpio_rst;
	/* internal */
	uint8_t  fullscreen;
};

#define SSD130x_NB_LINES   64
#define SSD130x_NB_PAGES   8
#define SSD130x_NB_COL     128

enum ssd130x_defs {
	SSD130x_DISP_OFF = 0,
	SSD130x_DISP_ON,
	SSD130x_DISP_RAM,
	SSD130x_DISP_BLANK,
	/* For reverse video */
	SSD130x_DISP_NORMAL,
	SSD130x_DISP_REVERSE,
	/* For scan dirrection */
	SSD130x_SCAN_TOP_BOTTOM,
	SSD130x_SCAN_BOTTOM_TOP,
	SSD130x_RIGHT_TO_LEFT,
	SSD130x_LEFT_TO_RIGHT,
	/* For display offset */
	SSD130x_MOVE_TOP,
	SSD130x_MOVE_BOTTOM,
	/* I2C or SPI */
	SSD130x_BUS_I2C,
	SSD130x_BUS_SPI,
};


#define SSD130x_DATA_ONLY       0x40
#define SSD130x_NEXT_BYTE_DATA  0xC0
#define SSD130x_NEXT_BYTE_CMD   0x80

/* Display controll */
#define SSD130x_CMD_CONTRAST           0x81
#define SSD130x_CMD_DISP_RAM           0xA4
#define SSD130x_CMD_DISP_NORAM         0xA5
#define SSD130x_CMD_DISP_NORMAL        0xA6
#define SSD130x_CMD_DISP_REVERSE       0xA7
#define SSD130x_CMD_DISP_OFF           0xAE
#define SSD130x_CMD_DISP_ON            0xAF

/* Scrolling controll */
#define SSD130x_CMD_SCROLL_RIGHT       0x26
#define SSD130x_CMD_SCROLL_LEFT        0x27
#define SSD130x_CMD_VSCROLL_RIGHT      0x29
#define SSD130x_CMD_VSCROLL_LEFT       0x2A
#define SSD130x_CMD_STOP_SCROLL        0x2E
#define SSD130x_CMD_START_SCROLL       0x2F
#define SSD130x_CMD_VSCROLL_REGION     0xA3
/* Data bytes for scrolling controll */
#define SSD130x_SCROLL_DATA_DUMMY  0x00
#define SSD130x_SCROLL_DATA_END    0xFF
#define SSD130x_SCROLL_DATA_START_PAGE(x) ((x) & 0x07)
#define SSD130x_SCROLL_DATA_END_PAGE(x)   ((x) & 0x07)
#define SSD130x_SCROLL_DATA_ROWS(x)   ((x) & 0x3F)
#define SSD130x_SCROLL_DATA_STEP(x) ((x) & 0x07)
/* Scroll steps definitions */
#define SSD130x_SCROLL_2_FRAMES    0x07
#define SSD130x_SCROLL_3_FRAMES    0x04
#define SSD130x_SCROLL_4_FRAMES    0x05
#define SSD130x_SCROLL_5_FRAMES    0x00
#define SSD130x_SCROLL_25_FRAMES   0x06
#define SSD130x_SCROLL_64_FRAMES   0x01
#define SSD130x_SCROLL_128_FRAMES  0x02
#define SSD130x_SCROLL_256_FRAMES  0x03

/* GDDRAM Adressing */
#define SSD130x_CMD_ADDR_TYPE          0x20
/* Data bytes for adressing mode election */
#define SSD130x_ADDR_TYPE_HORIZONTAL 0x00
#define SSD130x_ADDR_TYPE_VERTICAL   0x01
#define SSD130x_ADDR_TYPE_PAGE       0x02

/* GDDRAM Page adressing mode */
#define SSD130x_CMD_COL_LOW_NIBLE(x)   (0x00 + ((x) & 0x0F))
#define SSD130x_CMD_COL_HIGH_NIBLE(x)  (0x10 + ((x) & 0x0F))
#define SSD130x_CMD_PAGE_START_ADDR(x) (0xB0 + ((x) & 0x07))

/* GDDRAM horizontal or vertical addressing mode */
#define SSD130x_CMD_COL_ADDR           0x21
#define SSD130x_CMD_PAGE_ADDR          0x22
/* Data bytes for horizontal or vertical adressing mode */
#define SSD130x_ADDR_COL(x)       ((x) & 0x7F)
#define SSD130x_ADDR_PAGE(x)      ((x) & 0x07)

/* Charge pump */
#define SSD130x_EXT_VCC       0x01
#define SSD130x_INTERNAL_PUMP 0x00
#define SSD130x_CMD_CHARGE_PUMP    0x8D
#define SSD130x_CMD_CHARGE_EXT     0x10
#define SSD130x_CMD_CHARGE_INTERN  0x14

/* Hardware configuration */
#define SSD130x_CMD_START_LINE(x)      (0x40 + ((x) & 0x3F))
#define SSD130x_CMD_SEG0_MAP_RIGHT     0xA1
#define SSD130x_CMD_SEG0_MAP_LEFT      0xA0

/* Hardware configuration : Mux ratio */
#define SSD130x_CMD_SET_MUX            0xA8
/* Set mux ratio Data to N+1 (Values for N from 0 to 14 are invalid) */
#define SSD130x_MUX_DATA(x)       ((x) & 0x3F)  /* Reset is N=63 (64 mux) */

/* Hardware configuration : COM Scan */
#define SSD130x_CMD_COM_SCAN_NORMAL    0xC0  /* Reset mode : top to bottom */
#define SSD130x_CMD_COM_SCAN_REVERSE   0xC8  /* Bottom to top */
#define SSD130x_CMD_DISPLAY_OFFSET     0xD3
/* Data for display offset (COM shift) */
#define SSD130x_OFFSET_DATA(x)    ((x) & 0x3F)
#define SSD130x_CMD_COM_PIN_CONF       0xDA
/* Data for COM pins hardware configuration */
#define SSD130x_COM_SEQUENTIAL    (0x00 << 4)
#define SSD130x_COM_ALTERNATIVE   (0x01 << 4) /* Reset mode */
#define SSD130x_COM_NO_REMAP      (0x00 << 5) /* Reset mode */
#define SSD130x_COM_REMAP         (0x01 << 5)

/* Timing and driving scheme : Clock */
#define SSD130x_CMD_DISP_CLK_DIV       0xD5
#define SSD130x_CLK_DIV(x)        ((x) & 0x0F)  /* Set to N+1 (Default is 0+1) */
#define SSD130x_CLK_FREQ(x)       (((x) & 0x0F) << 4)  /* Reset is 0x80 */

/* Timing and driving scheme : Precharge */
#define SSD130x_CMD_SET_PRECHARGE      0xD9
#define SSD130x_PRECHARGE_PHASE1(x)    ((x) & 0x0F)  /* Default to 2, 0 is invalid */
#define SSD130x_PRECHARGE_PHASE2(x)    (((x) & 0x0F) << 4) /* Default to 2, 0 is invalid */

/* Timing and driving scheme : Voltage */
#define SSD130x_CMD_VCOM_LEVEL         0xDB
#define SSD130x_VCOM_065    0x00
#define SSD130x_VCOM_077    0x20
#define SSD130x_VCOM_083    0x30

/* NO-OP */
#define SSD130x_CMD_NOP                0xE3

/* Status register read */
#define SSD130x_STATUS_ON     (0x01 << 6)


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
int ssd130x_send_command(struct oled_display* conf, uint8_t cmd, uint8_t* data, uint8_t len);


/* Set memory adressing mode (0x20)
 * There are 3 different memory addressing mode in SSD130x: page addressing
 *  mode, horizontal addressing mode and vertical addressing mode
 *  - SSD130x_ADDR_TYPE_HORIZONTAL
 *  - SSD130x_ADDR_TYPE_VERTICAL
 *  - SSD130x_ADDR_TYPE_PAGE
 * This command sets the way of memory addressing into one of the above three
 *   modes.
 * (Refer to chapter 10.1.3 of SSD1306 manual)
 *
 * Note : This driver does not implemment and support page adressing. All page
 *  addressing commands are not supported.
 */
int ssd130x_set_mem_addressing_mode(struct oled_display* conf, uint8_t type);

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
int ssd130x_set_column_address(struct oled_display* conf, uint8_t col_start, uint8_t col_end);

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
int ssd130x_set_page_address(struct oled_display* conf, uint8_t page_start, uint8_t page_end);

/* Set contrast (0x81)
 * The contrast value is between 0 and 256
 */
int ssd130x_set_contrast(struct oled_display* conf);

/* Set display ON from GDDRAM (0xA4) or regardless of GDDRAM (0xA5)
 * use_ram is either SSD130x_DISP_RAM or SSD130x_DISP_BLANK
 */
int ssd130x_set_display_on(struct oled_display* conf, uint8_t use_ram);

/* Set display to normal or reverse video
 * status is either SSD130x_DISP_NORMAL or SSD130x_DISP_REVERSE
 */
int ssd130x_display_video_reverse(struct oled_display* conf);

/* Set display ON/OFF
 * status is either SSD130x_DISP_ON or SSD130x_DISP_OFF
 */
int ssd130x_display_power(struct oled_display* conf, uint8_t status);

/* Set scan direction (0xC0 / 0xC8)
 * Scan direction is either SSD130x_SCAN_TOP_BOTTOM or SSD130x_SCAN_BOTTOM_TOP
 */
int ssd130x_set_scan_direction(struct oled_display* conf);

/* Set read direction (0xA0 / 0xA1)
 * Scan direction is either SSD130x_CMD_SEG0_MAP_RIGHT or SSD130x_CMD_SEG0_MAP_LEFT
 */
int ssd130x_set_read_direction(struct oled_display* conf);

/* Set display offset
 * Move the display up or down by a given number of lines
 * FIXME : Check influence of COM output scan direction.
 */
int ssd130x_set_display_offset(struct oled_display* conf, uint8_t dir, uint8_t nb_lines);



/***************************************************************************** */
/* Init */
int ssd130x_display_on(struct oled_display* conf);
int ssd130x_display_off(struct oled_display* conf);


#define GDDRAM_SIZE   (128 * 8)
/***************************************************************************** */
/* Data */

/* Our internal buffer for the whole display.
 * Graphical data starts at byte 4. The first four bytes are here for temporary
 *  storage of display data during I2C frame transfer.
 */
int ssd130x_send_data(struct oled_display* conf, uint8_t* start, uint16_t len);


/* Update what is really displayed */
int ssd130x_display_full_screen(struct oled_display* conf);
int ssd130x_update_tile(struct oled_display* conf, uint8_t x0, uint8_t y0);
int ssd130x_update_region(struct oled_display* conf,
							uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
int ssd130x_update_modified(struct oled_display* conf);

#endif /* EXTDRV_SSD130X_OLED_DRIVER_H */
