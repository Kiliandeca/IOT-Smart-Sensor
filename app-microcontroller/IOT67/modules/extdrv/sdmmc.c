/****************************************************************************
 *   extdrv/sdmmc.c
 *
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
 * Copyright 2012 Gabriel Huau <contact@huau-gabriel.fr>
 *
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
 *************************************************************************** */

#include "core/system.h"
#include "lib/string.h"
#include "lib/errno.h"
#include "lib/crc_ccitt.h"
#include "lib/utils.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"

#include "extdrv/sdmmc.h"




/***************************************************************************** */
/*          Support for SD/MMC cards access over SSP/SPI bus                   */
/***************************************************************************** */


static inline void sdmmc_cs_activate(const struct sdmmc_card* mmc)
{
	gpio_clear(mmc->chip_select);
}
static inline void sdmmc_cs_release(const struct sdmmc_card* mmc)
{
	gpio_set(mmc->chip_select);
}

/* Wait for MMC Card to be ready.
 * return -EBUSY on timeout.
 * Note : The SPI Bus mutex must be held by the calling function.
 * FIXME : handle timeout with sleep()
 */
static int sdmmc_wait_for_ready(const struct sdmmc_card* mmc, uint8_t token)
{
	int i = 0;
	uint8_t val = 0;

	while (i++ < MMC_MAX_TIMEOUT) {
		val = (uint8_t)spi_transfer_single_frame(mmc->ssp_bus_num, 0xFF);
		if (token != 0) {
			if (val == token)
				return val;
		} else {
			if (val != 0)
				return val;
		}
	}

	return -EBUSY;
}

uint8_t sdmmc_crc7(uint8_t* data, int len)
{
	int i, idx;
	uint8_t crc = 0, val = 0;

	for (idx = 0; idx < len; idx++) {
		val = data[idx];
		for (i = 0; i < 8; i++) {
			crc <<= 1;
			if ((val & 0x80) ^ (crc & 0x80)) {
				crc ^= 0x09;
			}
			val <<= 1;
		}
	}
	crc = (crc << 1) | 1;
	return crc;
}


/* Send a command to the card.
 * The len parameter indicates the response type (and is the length of the buffer) :
 *   R1 : len = 0
 *   R1b : len = -1
 *   R2 : len = 1
 *   R3 or R7 : len = 4
 * Note : The SPI Bus mutex must be held by the calling function.
 * Return R1 or (R1 | 0x0100) on timeout.
 */
static int sdmmc_send_command(const struct sdmmc_card* mmc, uint8_t index, uint32_t arg, uint8_t* buf, int len)
{
	uint8_t mmc_command[MMC_CMD_SIZE];
	int i = 0;
	uint8_t r1 = 0;

	if ((len > 0) && (buf == NULL)) {
		return -EINVAL;
	}

	/* Command buffer */
	mmc_command[0] = 0x40 | index;
	mmc_command[1] = (arg >> 24) & 0xFF;
	mmc_command[2] = (arg >> 16) & 0xFF;
	mmc_command[3] = (arg >> 8) & 0xFF;
	mmc_command[4] = arg & 0xFF;
	mmc_command[5] = sdmmc_crc7(mmc_command, 5);

	/* Send command */
	spi_transfer_multiple_frames(mmc->ssp_bus_num, mmc_command, NULL, MMC_CMD_SIZE, 8);

	/* Get R1 */
	for (i = 0; i < 8; i++) {
		r1 = (uint8_t)spi_transfer_single_frame(mmc->ssp_bus_num, 0xFF);
		if (r1 != 0xFF) {
			break;
		}
	}
	/* Maybe get R2 ? */
	if (len == 1) {
		*buf = (uint8_t)spi_transfer_single_frame(mmc->ssp_bus_num, 0xFF);
	}
	/* Error ? Do not wait for the data in case of error */
	if (r1 > MMC_R1_IN_IDLE_STATE) {
		return r1;
	}
	/* Wait for end of busy state ? (The R1 + busy case) */
	if (len == -1) {
		int ret = sdmmc_wait_for_ready(mmc, 0);
		if (ret == -EBUSY) {
			return (r1 | 0x0100);
		}
	}
	if (len > 1) {
		/* Receive response data  */
		spi_transfer_multiple_frames(mmc->ssp_bus_num, NULL, buf, len, 8);
	}

	return r1;
}

static int sdmmc_send_app_command(const struct sdmmc_card* mmc, uint8_t index, uint32_t arg)
{
	int r1 = 0;

	/* Send APP_CMD (CMD55) first */
	r1 = sdmmc_send_command(mmc, MMC_APP_CMD, 0, NULL, 0);
	if (r1 > MMC_R1_IN_IDLE_STATE) {
		return r1;
	}
	return sdmmc_send_command(mmc, index, arg, NULL, 0);
}


int sdmmc_init(struct sdmmc_card* mmc)
{
	int r1 = 0, ret = 0;
	uint8_t buf[MMC_CMD_SIZE];

	config_gpio(&(mmc->chip_select), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 1);
	mmc->card_type = MMC_CARDTYPE_UNKNOWN;

	/* Get SPI Bus */
	spi_get_mutex(mmc->ssp_bus_num);
	sdmmc_cs_activate(mmc);

	/* Send CMD0 (RESET or GO_IDLE_STATE) */
	r1 = sdmmc_send_command(mmc, MMC_GO_IDLE_STATE, 0, NULL, 0);
	if (r1 > MMC_R1_IN_IDLE_STATE) {
		ret = -EIO; /* This one should have succeded even without a card inserted ... */
		goto init_release_bus;
	}
	msleep(1);

	/* Send CMD8, check the card type
	 * CMD8 is the Send interface Condition Command.
	 *  0x01 is "2.7V to 3.6V"
	 *  0xAA is a "check pattern", which the card must send back.
	 */
	r1 = sdmmc_send_command(mmc, MMC_SEND_IF_COND, 0x1AA, buf, 4);
	if (r1 > MMC_R1_IN_IDLE_STATE) {
		ret = -ENODEV; /* No card inserted */
		goto init_release_bus;
	}
	if (r1 == MMC_R1_IN_IDLE_STATE) {
		/* Card is SD V2 */
		if (buf[2] != 0x01 || buf[3] != 0xAA) {
			ret = -EREMOTEIO;
			mmc->card_type = MMC_CARDTYPE_UNKNOWN;
			goto init_release_bus;
		}
		mmc->card_type = MMC_CARDTYPE_SDV2_SC;
	} else {
		mmc->card_type = MMC_CARDTYPE_SDV1; /* Card is 2.1mm thick SD Card */
	}

	/* Read OCR (CMD58) */
	if (mmc->card_type == MMC_CARDTYPE_SDV2_SC) {
		r1 = sdmmc_send_command(mmc, MMC_READ_OCR, 0, buf, 4);
		if (r1 > MMC_R1_IN_IDLE_STATE) {
			ret = -EREMOTEIO;
			goto init_release_bus;
		}
	}

	/* Send first ACMD41 command */
	r1 = sdmmc_send_app_command(mmc, MMC_SD_SEND_OP_COND, SDHC_SUPPORT_OK);
	if (r1 > MMC_R1_IN_IDLE_STATE) {
		ret = -EREMOTEIO;
	}

init_release_bus:
	/* Release SPI Bus */
	sdmmc_cs_release(mmc);
	spi_release_mutex(mmc->ssp_bus_num);

	return ret;
}

int sdmmc_init_wait_card_ready(struct sdmmc_card* mmc)
{
	int r1 = 0;

	/* Get SPI Bus */
	spi_get_mutex(mmc->ssp_bus_num);
	sdmmc_cs_activate(mmc);

	r1 = sdmmc_send_app_command(mmc, MMC_SD_SEND_OP_COND, SDHC_SUPPORT_OK);

	/* Release SPI Bus */
	sdmmc_cs_release(mmc);
	spi_release_mutex(mmc->ssp_bus_num);

	return r1;
}

int sdmmc_init_end(struct sdmmc_card* mmc)
{
	int r1 = 0, ret = 0;
	uint8_t buf[MMC_CMD_SIZE];

	/* Get SPI Bus */
	spi_get_mutex(mmc->ssp_bus_num);
	sdmmc_cs_activate(mmc);

	/* Read OCR (CMD58) */
	if (mmc->card_type == MMC_CARDTYPE_SDV2_SC) {
		r1 = sdmmc_send_command(mmc, MMC_READ_OCR, 0, buf, 4);
		if (r1 > MMC_R1_IN_IDLE_STATE) {
			ret = -EREMOTEIO;
			goto end_init_release_bus;
		}
		mmc->card_type = (buf[0] & 0x40) ? MMC_CARDTYPE_SDV2_HC : MMC_CARDTYPE_SDV2_SC;
	}

	/* Set block length */
	if (mmc->block_size > MMC_MAX_SECTOR_SIZE) {
		mmc->block_size = MMC_MAX_SECTOR_SIZE;
		mmc->block_shift = (31 - clz(mmc->block_size));
	} else {
		/* Force mmc->block_size to a power of 2 */
		mmc->block_shift = (31 - clz(mmc->block_size));
		mmc->block_size = (0x01 << mmc->block_shift);
	}
	r1 = sdmmc_send_command(mmc, MMC_SET_BLOCKLEN, mmc->block_size, NULL, 0);
	if (r1 > MMC_R1_IN_IDLE_STATE) {
		mmc->card_type = MMC_CARDTYPE_UNKNOWN;
	}

end_init_release_bus:
	/* Release SPI Bus */
	sdmmc_cs_release(mmc);
	spi_release_mutex(mmc->ssp_bus_num);

	return ret;
}


/* Read one block of data.
 * Returns -EINVAL on arguments error, -ENODEV on command error,
 *         -EBUSY on timeout, -EIO on CRC error,
 *         or 0 on success
 */
#define TMPBUF_SIZE 64
int sdmmc_read_block(const struct sdmmc_card* mmc, uint32_t block_number, uint8_t* buffer)
{
	uint16_t crc = 0x0000;
	uint16_t sd_crc = 0xFFFF;
	int ret = 0;

	if ((buffer == NULL) || (mmc->card_type == MMC_CARDTYPE_UNKNOWN)) {
		return -EINVAL;
	}

	/* Garbage for the SPI out data */
	memset(buffer, 0xFF, mmc->block_size);

	/* Non SDHC cards use address and not block number */
	if (mmc->card_type != MMC_CARDTYPE_SDV2_HC) {
		block_number = (block_number << mmc->block_shift);
	}

	/* Get SPI Bus */
	spi_get_mutex(mmc->ssp_bus_num);
	sdmmc_cs_activate(mmc);

	ret = sdmmc_send_command(mmc, MMC_READ_SINGLE_BLOCK, block_number, NULL, 0);
	if (ret != MMC_R1_NO_ERROR) {
		ret = -ENODEV;
		goto read_release;
	}

	/* Wait for start of Data */
	ret = sdmmc_wait_for_ready(mmc, MMC_START_DATA_BLOCK_TOCKEN);
	if (ret != MMC_START_DATA_BLOCK_TOCKEN) {
		ret = -EBUSY;
		goto read_release;
	}

	/* Read data, interresting part */
	ret = spi_transfer_multiple_frames(mmc->ssp_bus_num, NULL, buffer, mmc->block_size, 8);
	/* Compute CRC of this part */
	crc = crc_ccitt(0x0000, buffer, mmc->block_size);
	/* Read data, remaining part */
	if ((mmc->card_type == MMC_CARDTYPE_SDV2_HC) && (mmc->block_size != MMC_MAX_SECTOR_SIZE)) {
		char tmpbuf[TMPBUF_SIZE];
		int size = (MMC_MAX_SECTOR_SIZE - mmc->block_size);
		do {
			int tmp_size = TMPBUF_SIZE;
			memset(tmpbuf, 0, TMPBUF_SIZE);
			if (tmp_size > size) {
				tmp_size = size;
			}
			/* Read data - remaining part ... and drop it (null input but non null output buffer) */
			ret = spi_transfer_multiple_frames(mmc->ssp_bus_num, NULL, tmpbuf, tmp_size, 8);
			/* Update CRC with this part */
			crc = crc_ccitt(crc, (uint8_t*)tmpbuf, tmp_size);
			size -= tmp_size;
		} while (size > 0);
	}
	/* Read CRC (received in network endianness) */
	ret = spi_transfer_multiple_frames(mmc->ssp_bus_num, NULL, (uint8_t*)(&sd_crc), 2, 8);
	sd_crc = (uint16_t)ntohs(sd_crc);
	if (crc != sd_crc) {
		ret = -EIO;
		goto read_release;
	}

	/* Wait for card ready */
	ret = sdmmc_wait_for_ready(mmc, 0xFF);
	if (ret != 0xFF) {
		ret = -EBUSY;
		goto read_release;
	}

	ret = 0;

read_release:
	/* Release SPI Bus */
	sdmmc_cs_release(mmc);
	spi_release_mutex(mmc->ssp_bus_num);
	return ret;
}



/* Write one block of data.
 * This routine does not pre-erase the block, so if the user did not pre-erase the
 *   corresponding block then the write takes longer.
 * Returns -EINVAL on arguments error, -ENODEV on command error,
 *         -ECOMM on response error, -EIO on CRC error, -EPERM on write error
 *         or 0 on success
 */
int sdmmc_write_block(const struct sdmmc_card* mmc, uint32_t block_number, uint8_t* buffer)
{
	uint16_t crc = 0xFFFF;
	int ret = 0;

	if ((buffer == NULL) || (mmc->card_type == MMC_CARDTYPE_UNKNOWN)) {
		return -EINVAL;
	}

	if (mmc->card_type != MMC_CARDTYPE_SDV2_HC) {
		block_number = (block_number << mmc->block_shift);
	}

	/* Get SPI Bus */
	spi_get_mutex(mmc->ssp_bus_num);
	sdmmc_cs_activate(mmc);

	ret = sdmmc_send_command(mmc, MMC_WRITE_SINGLE_BLOCK, block_number, NULL, 0);
	if (ret != MMC_R1_NO_ERROR) {
		ret = -ENODEV;
		goto write_release;
	}

	/* Send start of Data token */
	ret = (uint8_t)spi_transfer_single_frame(mmc->ssp_bus_num, MMC_START_DATA_BLOCK_TOCKEN);

	/* Send interresting data */
	ret = spi_transfer_multiple_frames(mmc->ssp_bus_num, buffer, NULL, mmc->block_size, 8);
	/* Compute CRC of this part */
	crc = crc_ccitt(0x0000, buffer, mmc->block_size);
	/* Send data, remaining part */
	if ((mmc->card_type == MMC_CARDTYPE_SDV2_HC) && (mmc->block_size != MMC_MAX_SECTOR_SIZE)) {
		int size = (MMC_MAX_SECTOR_SIZE - mmc->block_size);
		do {
			int tmp_size = mmc->block_size;
			if (tmp_size > size) {
				tmp_size = size;
			}
			/* Send data - remaining part ... using the same data again */
			ret = spi_transfer_multiple_frames(mmc->ssp_bus_num, buffer, NULL, tmp_size, 8);
			/* Update CRC with this part */
			crc = crc_ccitt(crc, buffer, tmp_size);
			size -= tmp_size;
		} while (size > 0);
	}
	/* Send CRC (in network endianness) */
	crc = (uint16_t)ntohs(crc);
	ret = spi_transfer_multiple_frames(mmc->ssp_bus_num, (uint8_t*)(&crc), NULL, 2, 8);

	/* Get Data response tocken */
	ret = (uint8_t)spi_transfer_single_frame(mmc->ssp_bus_num, 0xFF);
	if (!MMC_IS_WRITE_RESPONSE_TOKEN(ret)) {
		ret = -ECOMM;
		goto write_release;
	}
	ret = MMC_WRITE_RESPONSE_TOKEN(ret);
	if (ret != MMC_WRITE_RESPONSE_OK) {
		if (ret == MMC_WRITE_RESPONSE_CRC_ERR) {
			ret = -EIO;
		} else {
			ret = -EPERM;
		}
		goto write_release;
	}

	/* Wait for card ready */
	ret = sdmmc_wait_for_ready(mmc, 0xFF);
	if (ret != 0xFF) {
		ret = -EBUSY;
		goto write_release;
	}

	ret = 0;

write_release:
	/* Release SPI Bus */
	sdmmc_cs_release(mmc);
	spi_release_mutex(mmc->ssp_bus_num);
	return ret;
}



