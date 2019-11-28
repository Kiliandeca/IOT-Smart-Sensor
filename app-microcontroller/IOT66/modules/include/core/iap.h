/****************************************************************************
 *   core/iap.h
 *
 * Copyright 2015 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef CORE_IAP_H
#define CORE_IAP_H

#include "lib/stdint.h"


/*******************************************************************************/
/*            In Application Programming ROM based routines                    */
/*******************************************************************************/

/* Provide access to IAP ROM based routines.
 * This is the only access to User information block, and allows in-application re-programming
 *   of the micro-controller (for bootloaders, drivers, loadable RTOS tasks, ....)
 */

#define START_APP_ADDRESS (0x5000)
#define INITIAL_SP      (*(uint32_t *)(START_APP_ADDRESS))
#define RESET_HANDLER   (*(uint32_t *)(START_APP_ADDRESS + 4))
/* CRP Handling */
#define CRP_ADDRESS    (0x000002FC)
#define NO_ISP   (0x4E697370)
#define CRP1     (0x12345678)
#define CRP2     (0x87654321)
#define CRP3     (0x43218765)
#define IS_CRP_VALUE(v)  ((v==NO_ISP) || (v==CRP1) || (v==CRP2) || (v==CRP3))

#define LPC12XX_SECTOR_SIZE  (0x1000)
#define LPC1224_101_NB_SECTOR    (8)
#define LPC1224_121_NB_SECTOR    (12)
#define LPC1225_301_NB_SECTOR    (16)
#define LPC1225_321_NB_SECTOR    (20)
#define LPC1226_301_NB_SECTOR    (24)
#define LPC1227_301_NB_SECTOR    (32)
#define LPC12XX_END_FLASH(version)    (((version) ## _NB_SECTOR) * LPC12xx_SECTOR_SIZE)



/* Return values */
enum iap_status {
	IAP_STATUS_CMD_SUCCESS = 0,
	IAP_STATUS_INVALID_COMMAND,
	IAP_STATUS_SRC_ADDR_ERROR,
	IAP_STATUS_DST_ADDR_ERROR,
	IAP_STATUS_SRC_ADDR_NOT_MAPPED,
	IAP_STATUS_DST_ADDR_NOT_MAPPED,
	IAP_STATUS_COUNT_ERROR,
	IAP_STATUS_INVALID_SECTOR,
	IAP_STATUS_SECTOR_NOT_BLANK,
	IAP_STATUS_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
	IAP_STATUS_COMPARE_ERROR,
	IAP_STATUS_BUSY,
};

/*******************************************************************************/
/* Direct access to IAP function */

/* Erase some pages from the user information block.
 * Page numbers may be 0, 1 or 2 for the LPC122x.
 * Provide the same page number as start and end to erase a single page.
 * Of course, end page number MUST be higher than (or equal to) start page number.
 * There is no way to erase only parts of a page.
 * Interrputs are disabled during this operation.
 */
int iap_erase_info_page(uint32_t start_page, uint32_t end_page);


/* Prepare sectors from the programm flash memory for erasing or writting
 * Sectors numbers start at 0. A flash sector size is 4kB (0x1000 bytes)
 * Provide the same sector number as start and end to prepare a single sector.
 * Of course, end sector number MUST be higher than (or equal to) start sector number.
 */
int iap_prepare_flash(uint32_t start_sector, uint32_t end_sector);

/* Erase full flash sectors from the programm memory
 * Sectors numbers start at 0. A flash sector size is 4kB (0x1000 bytes)
 * Provide the same sector number as start and end to erase a single sector.
 * Of course, end sector number MUST be higher than (or equal to) start sector number.
 * Use iap_erase_flash_pages() to erase a single page within a sector.
 * A sector must be prepared for writing before the erase operation.
 * Interrputs are disabled during this operation.
 */
int iap_erase_flash_sectors(uint32_t start_sector, uint32_t end_sector);

/* Erase pages  from the programm memory
 * Page numbers start at 0. A flash page size is 512 bytes.
 * Provide the same page number as start and end to erase a single page.
 * Of course, end page number MUST be higher than (or equal to) start page number.
 * There is no way to erase only parts of a page.
 * The sector in which the page reside must be prepared for writing before the page erase
 *    operation.
 * Interrputs are disabled during this operation.
 */
int iap_erase_flash_pages(uint32_t start_page, uint32_t end_page);


/* Copy some data from RAM to Flash.
 * When writting to the programm flash memory, the sectors must be prepared for writing
 *   before the copy operation.
 * Both dest and src parameters must be aligned to 4 bytes boundary and size must be a
 *   multiple of 4.
 * dest is the destination address and must be the address of some flash memory.
 * ser is the source address and must be the address of some RAM memory.
 * The sector or page in dest must have been erased before writing and no writing operation
 *   performed between (dest) and (dest+size) addresses. (Writting in other parts of the
 *   page or sector is OK)
 * Interrputs are disabled during this operation.
 */
int iap_copy_ram_to_flash(uint32_t dest, uint32_t src, uint32_t size);


/* Return the part ID */
uint32_t iap_read_part_id(void);

/* Copy the whole unique id table (four 32bits words) in the memory pointed by uid_table.
 * uid_table must have enougth room for the whole unique id table.
 * Returns the IAP status.
 */
uint32_t iap_read_unique_id(uint32_t* uid_table);



/*******************************************************************************/
/* IAP Helpers */

/* Relocate the vector table so that the first sector of the internal flash can be erased
 * and written
*/
void relocate_vector_table(void);

/* Erase one sector, given a flash address
 * return 0 on success.
 */
int flash_erase_sector(uint32_t addr);

/* Flash a binary image chunk to flash.
 * Flash must have been erased first.
 * return 0 on success.
 */
int flash_program_page (uint32_t addr, uint32_t sz, unsigned char *buf);

#endif /* CORE_IAP_H */

