/****************************************************************************
 *   extdrv/eeprom.c
 *
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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
#include "core/pio.h"
#include "lib/errno.h"
#include "lib/string.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "extdrv/eeprom.h"

/* FIXME : All of this will have to be re-written with an eeprom definition structure
 * This will have the address, bus number, and eeprom type, in order to remove the static
 *   variables and hardcoded bus number from this code.
 */

/***************************************************************************** */
/*          Read and Write for eeprom                                          */
/***************************************************************************** */

/* NOTE : This code does automatic detection of the eeprom type/size and thus does not
 *        support multiple eeproms on the same I2C bus
 */

/* Config */
/* Small eeprom : up to 2k bytes. These use a segment address on the lower three bits
 *   of the address byte, and thus reply on 8 consecutive addresses */
#define EEPROM_ID_SMALL_ADDR_1  0xA0
#define EEPROM_ID_SMALL_ADDR_2  0xA2
#define EEPROM_ID_SMALL_I2C_SIZE  1024
#define EEPROM_ID_SMALL_PAGE_SIZE 16
/* Big eeprom : from 4k bytes and above : These use two address bytes, and the three
 *   physical address pins are used to set the chip address. */
#define EEPROM_ID_BIG_I2C_SIZE  16*1024
#define EEPROM_ID_BIG_PAGE_SIZE 64


enum i2c_eeprom_type {
	EEPROM_TYPE_NONE = 0,
	EEPROM_TYPE_SMALL,
	EEPROM_TYPE_BIG,
};

/* Detect the eeprom size */
int eeprom_detect(uint8_t eeprom_addr)
{
	int ret = 0;
	char cmd_buf[1] = { EEPROM_ID_SMALL_ADDR_1, };

	/* Look for small eeproms first, only these would answer on all addresses */
	if (eeprom_addr == EEPROM_ID_SMALL_ADDR_1) {
		cmd_buf[0] = EEPROM_ID_SMALL_ADDR_2;
	}
	ret = i2c_read(0, cmd_buf, 1, NULL, NULL, 0);
	if (ret == 0) {
		return EEPROM_TYPE_SMALL;
	}

	/* No small eeprom ... look for big ones */
	cmd_buf[0] = eeprom_addr;
	ret = i2c_read(0, cmd_buf, 1, NULL, NULL, 0);
	if (ret == 0) {
		return EEPROM_TYPE_BIG;
	}

	if (ret > 0) {
		return -EIO;
	} else if (ret == -EREMOTEIO) {
		return EEPROM_TYPE_NONE; /* No module */
	}
	return ret; /* Error or module size */
}

int get_eeprom_type(uint8_t eeprom_addr)
{
	static int eeprom_type = -1;

	if (eeprom_type >= 0) {
		return eeprom_type; /* No need to check again */
	}

	eeprom_type = eeprom_detect(eeprom_addr);
	if (eeprom_type <= 0) {
		return -1;
	}
	return eeprom_type;
}


/* EEPROM Read
 * Performs a non-blocking read on the eeprom.
 *   address : data offset in eeprom.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes read. On error, returns a negative
 *   integer equivalent to errors from glibc.
 */
#define CMD_BUF_SIZE 4
int eeprom_read(uint8_t eeprom_addr, uint32_t offset, void *buf, size_t count)
{
	int ret = 0;
	char cmd_buf[CMD_BUF_SIZE];
	char ctrl_buf[CMD_BUF_SIZE] = { I2C_CONT, I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };
	int eeprom_type = 0;

	eeprom_type = get_eeprom_type(eeprom_addr);

	/* Read the requested data */
	switch (eeprom_type) {
		case EEPROM_TYPE_SMALL:
			cmd_buf[0] = EEPROM_ID_SMALL_ADDR_1 | ((offset & 0x700) >> 7);
			cmd_buf[1] = offset & 0xFF;
			cmd_buf[2] = EEPROM_ID_SMALL_ADDR_1 | 0x01;
			ret = i2c_read(0, cmd_buf, CMD_BUF_SIZE - 1, ctrl_buf + 1, buf, count);
			break;
		case EEPROM_TYPE_BIG:
			cmd_buf[0] = eeprom_addr;
			cmd_buf[1] = ((offset & 0xFF00) >> 8);
			cmd_buf[2] = offset & 0xFF;
			cmd_buf[3] = (eeprom_addr | 0x01);
			ret = i2c_read(0, cmd_buf, CMD_BUF_SIZE, ctrl_buf, buf, count);
			break;
		default:
			ret = -1;
			break;
	}

	return ret;
}


/* EEPROM Write
 * Performs a non-blocking write on the eeprom.
 *   address : data offset in eeprom.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes written. On error, returns a negative
 *   integer equivalent to errors from glibc.
 */
#define CMD_SIZE_SMALL 2
#define CMD_SIZE_BIG 3
#define MAX_CMD_SIZE CMD_SIZE_BIG
#define EEPROM_ID_MAX_PAGE_SIZE EEPROM_ID_BIG_PAGE_SIZE
int eeprom_write(uint8_t eeprom_addr, uint32_t offset, const void *buf, size_t count)
{
	int ret = 0;
	uint8_t cmd_size = CMD_SIZE_BIG, page_size = EEPROM_ID_BIG_PAGE_SIZE;
	int write_count = 0, size = 0;
	char cmd[MAX_CMD_SIZE];
	char full_buff[(EEPROM_ID_MAX_PAGE_SIZE + MAX_CMD_SIZE)];
	int eeprom_type = 0;

	eeprom_type = get_eeprom_type(eeprom_addr);

	switch (eeprom_type) {
		case EEPROM_TYPE_SMALL:
			cmd_size = CMD_SIZE_SMALL;
			page_size = EEPROM_ID_SMALL_PAGE_SIZE;
			break;
		case EEPROM_TYPE_BIG:
			/* already configured */
			/* cmd_size = CMD_SIZE_BIG; */
			/* page_size = EEPROM_ID_BIG_PAGE_SIZE; */
			break;
		default:
			ret = -1;
			write_count = count + 1; /* skip the while loop, but return error */
			break;
	}
	while (write_count < count) {
		switch (eeprom_type) {
			case EEPROM_TYPE_SMALL:
				cmd[0] = EEPROM_ID_SMALL_ADDR_1 | ((offset & 0x700) >> 7);
				cmd[1] = offset & 0xFF;
				break;
			case EEPROM_TYPE_BIG:
				cmd[0] = eeprom_addr;
				cmd[1] = ((offset & 0xFF00) >> 8);
				cmd[2] = offset & 0xFF;
				break;
		}
		/* make partial first write to allign to page boundaries */
		if (offset & (page_size - 1)) {
			size = (page_size - (offset & (page_size - 1)));
		} else {
			size = page_size;
		}
		if (size > (count - write_count))
			size = (count - write_count);
		offset += size;
		memcpy(full_buff, cmd, cmd_size);
		memcpy(full_buff + cmd_size, buf + write_count, size);
		ret = i2c_write(0, full_buff, (cmd_size + size), NULL);

		if (ret != (cmd_size + size)) {
			break;
		}
		/* Wait for page write completion : The device does not acknoledge anything during
		 * page write, perform page writes with no data, until it returns 1 */
		do {
			ret = i2c_write(0, full_buff, 1, NULL);
		} while (ret != 1);

		write_count += size;
	}

	if (write_count != count)
		return ret;
	return write_count;
}


