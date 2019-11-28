/****************************************************************************
 *   core/iap.c
 *
 *
 * Copyright 2015 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 * This file is derived from Work covered by the Apache 2.0 licence.
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Original sources should be found there :
 *     https://github.com/mbedmicro/CMSIS-DAP.git
 * Original file name is bootloader/hal/TARGET_NXP/TARGET_LPC11U35/flash_hal.c
 *
 * Original copyright notice :
 *    CMSIS-DAP Interface Firmware
 *    Copyright (c) 2009-2013 ARM Limited
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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
 *****************************************************************************/

#include "lib/stdint.h"
#include "core/iap.h"


/* IAP - In-Application Programming
 * Driver for the IAP interface of the LPC122x.
 * This interface allows In-Application programming of the internal flash memory of the LPC122x
 *   microcontroller.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */

static uint32_t get_sector_number(uint32_t addr)
{
	uint32_t sector = 0;

	sector = addr >> 12;  /* 4kB Sector */
	if (sector >= 0x10) {
		sector = 0x0E + (sector >> 3); /* 32kB Sector */
	}

	return sector;
}

/* Erase one sector, given a flash address
 * return 0 on success.
 */
int flash_erase_sector(uint32_t addr)
{
	uint32_t sector = get_sector_number(addr);
	int ret = 0;

	/* Prepare sector for erase */
	ret = iap_prepare_flash(sector, sector);
	if (ret != IAP_STATUS_CMD_SUCCESS) {
		return ret;
	}
	/* Erase sector */
	return iap_erase_flash_sectors(sector, sector);
}

/* Flash a binary image chunk to a sector. */
int flash_program_page(uint32_t addr, uint32_t size, unsigned char *buf)
{
	uint32_t sector = 0;
	int ret = 0;

	/* If this goes to the beginning of the Flash image, check that the image is valid. */
	if (addr == 0) {
		uint32_t crp = *((uint32_t *)(buf + CRP_ADDRESS));
		uint32_t checksum = 0;

		if (IS_CRP_VALUE(crp)) {
			/* CRP is enabled, exit. */
			return -1;
		}

		/* Compute a valid user code */
		checksum = *((uint32_t *)(buf + 0x00)) + *((uint32_t *)(buf + 0x04)) +
					*((uint32_t *)(buf + 0x08)) + *((uint32_t *)(buf + 0x0C)) +
					*((uint32_t *)(buf + 0x10)) + *((uint32_t *)(buf + 0x14)) +
					*((uint32_t *)(buf + 0x18));
		*((uint32_t *)(buf + 0x1C)) = 0 - checksum;
	}

	sector = get_sector_number(addr);

	/* Prepare sector for write */
	ret = iap_prepare_flash(sector, sector);
	if (ret != IAP_STATUS_CMD_SUCCESS) {
		return ret;
	}

	/* FIXME : in original code, size was hardcoded to 1024 */
	return iap_copy_ram_to_flash(addr, (uint32_t)buf, size);
}

