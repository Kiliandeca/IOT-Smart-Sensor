/****************************************************************************
 *  extdrv/sdmmc.h
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
 * Copyright 2012 Gabriel Huau <contact@huau-gabriel.fr>
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
#ifndef EXTDRV_SDMMC_H
#define EXTDRV_SDMMC_H

#include "lib/stdint.h"
#include "core/pio.h"


/***************************************************************************** */
/*          Support for SD/MMC cards access over SSP/SPI bus                   */
/***************************************************************************** */


struct sdmmc_card {
	uint8_t ssp_bus_num;
	uint8_t card_type;
	uint16_t block_size;
	uint8_t block_shift;
	struct pio chip_select;
};

int sdmmc_init(struct sdmmc_card* mmc);
int sdmmc_init_wait_card_ready(struct sdmmc_card* mmc);
int sdmmc_init_end(struct sdmmc_card* mmc);

/* Read one block of data.
 * Return -ENODEV on error, -EBUSY on timeout, -EIO on CRC error, or 0 on success
 */
int sdmmc_read_block(const struct sdmmc_card* mmc, uint32_t block_number, uint8_t *buffer);
int sdmmc_write_block(const struct sdmmc_card* mmc, uint32_t block_number, uint8_t *buffer);


/* Card states and Operation modes */
/* Inactive operation mode */
#define MMC_OP_MODE_INACTIVE           0
#define MMC_CARD_STATE_INACTIVE        0
/* Identification operation mode */
#define MMC_OP_MODE_IDENTIFICATION     1
#define MMC_CARD_STATE_IDLE            1
#define MMC_CARD_STATE_READY           2
#define MMC_CARD_STATE_IDENTIFICATION  3
/* Data transfer mode */
#define MMC_OP_MODE_DATA_TRANSFER      4
#define MMC_CARD_STATE_STANDBY         4
#define MMC_CARD_STATE_TRANSFER        5
#define MMC_CARD_STATE_SENDING_DATA    6
#define MMC_CARD_STATE_RECEIVE_DATA    7
#define MMC_CARD_STATE_PROGRAMMING     8
#define MMC_CARD_STATE_DISCONNECT      9


/* Command definitions in SPI bus mode
 * Response type is R1 unless specified
 */
#define MMC_GO_IDLE_STATE           0
#define MMC_SEND_OP_COND            1
#define MMC_SWITCH_FUNC             6
#define MMC_SEND_IF_COND            8 /* R7 (R1 + 4 bytes) */
#define MMC_SEND_CSD                9
#define MMC_SEND_CID                10
#define MMC_STOP_TRANSMISSION       12 /* R1b (busy) */
#define MMC_SEND_STATUS             13 /* R2 (2 bytes) */
#define MMC_SET_BLOCKLEN            16
#define MMC_READ_SINGLE_BLOCK       17
#define MMC_READ_MULTIPLE_BLOCK     18
#define MMC_WRITE_SINGLE_BLOCK      24
#define MMC_WRITE_MULTIPLE_BLOCK    25
#define MMC_PROGRAMM_CSD            27
/* Write protect commands are unsupported by SDHC and SDXC cards */
#define MMC_SET_WRITE_PROTECT       28 /* R1b (busy) */
#define MMC_CLR_WRITE_PROTECT       29 /* R1b (busy) */
#define MMC_SEND_WRITE_PROTECT      30
#define ERASE_WR_BLK_START_ADDR     32
#define ERASE_WR_BLK_END_ADDR       33
#define MMC_ERASE                   38 /* R1b (busy) */
#define MMC_LOCK_UNLOCK             42
#define MMC_APP_CMD                 55
#define MMC_GEN_CMD                 56
#define MMC_READ_OCR                58 /* R3 (R1 + 4 bytes of OCR )*/
#define MMC_CRC_ON_OFF              59

/* Application specific commands supported by SD.
 * All these commands shall be preceded with APP_CMD (CMD55).
 */
#define MMC_SD_SEND_STATUS          13 /* R2 (2 bytes) */
#define MMC_SD_SEND_NUM_WR_BLOCKS   22
#define MMC_SD_SET_WR_BLK_ERASE_COUNT  23
#define MMC_SD_SEND_OP_COND         41
#define MMC_SD_SET_CLR_CARD_DETECT  42
#define MMC_SD_SEND_SCR             51

/* HCS bit */
#define SDHC_SUPPORT_OK   (0x01 << 30)


/* R1 response bit flag definition */
#define MMC_R1_NO_ERROR         0x00
#define MMC_R1_IN_IDLE_STATE    (0x01 << 0)
#define MMC_R1_ERASE_RESET      (0x01 << 1)
#define MMC_R1_ILLEGAL_CMD      (0x01 << 2)
#define MMC_R1_COM_CRC_ERROR    (0x01 << 3)
#define MMC_R1_ERASE_SEQ_ERROR  (0x01 << 4)
#define MMC_R1_ADDRESS_ERROR    (0x01 << 5)
#define MMC_R1_PARAMETER_ERROR  (0x01 << 6)
#define MMC_R1_MASK             0x7F

/* Memory card type definitions */
#define MMC_CARDTYPE_UNKNOWN        0
#define MMC_CARDTYPE_MMC            1   /* MMC */
#define MMC_CARDTYPE_SDV1           2   /* V1.x Standard Capacity SD card */
#define MMC_CARDTYPE_SDV2_SC        3   /* V2.0 or later Standard Capacity SD card */
#define MMC_CARDTYPE_SDV2_HC        4   /* V2.0 or later High/eXtended Capacity SD card */

/* The sector size is fixed to 512bytes in most applications. */
#define MMC_MAX_SECTOR_SIZE 512
#define MMC_SECTOR_BITS 9

/* Buffer CMD/DATA size */
#define MMC_CMD_SIZE    6
#define MMC_DATA_SIZE   512

#define MMC_MAX_TIMEOUT 10000


#define MMC_CMD8_CHECK_PATERN 0xAA
#define MMC_VOLTAGE_SELECT_3V3  0x01


#define MMC_START_DATA_BLOCK_TOCKEN  0xFE

#define MMC_IS_WRITE_RESPONSE_TOKEN(x)  (((x) & 0x11) == 0x01)
#define MMC_WRITE_RESPONSE_TOKEN(x)     (((x) & 0x0E) >> 1)
#define MMC_WRITE_RESPONSE_OK       2
#define MMC_WRITE_RESPONSE_CRC_ERR  5
#define MMC_WRITE_RESPONSE_WR_ERROR 6

#endif /* EXTDRV_SDMMC_H */


