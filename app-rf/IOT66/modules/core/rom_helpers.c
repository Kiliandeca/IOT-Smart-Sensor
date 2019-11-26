/****************************************************************************
 *   core/rom_helpers.c
 *
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

/* ROM helpers are functions avalable for the user which do not use space in the
 *  internal reprogrammable flash.
 * They are stored in the internal ROM memory and may be called using specific
 *  calls with defined parameters depending on the ROM call used.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */

#include "core/system.h"
#include "core/iap.h"

/*******************************************************************************/
/*            Integer division using ROM based division routines               */
/*******************************************************************************/

struct idiv_return {
	int quotient;
	int remainder;
};
struct uidiv_return {
	unsigned quotient;
	unsigned remainder;
};

struct lpc_rom_div_helpers {
	/* Signed integer division */
	int (*rom_sidiv)(int numerator, int denominator);
	/* Unsigned integer division */
	unsigned (*rom_uidiv)(unsigned numerator, unsigned denominator);
	/* Signed integer division with remainder */
	struct idiv_return (*rom_sidivmod)(int numerator, int denominator);
	/* Unsigned integer division with remainder */
	struct uidiv_return (*rom_uidivmod)(unsigned numerator, unsigned denominator);
};

static struct lpc_rom_div_helpers* rom_div_helpers;

/* Division (/) */
int __aeabi_idiv(int numerator, int denominator)
{
	return rom_div_helpers->rom_sidiv(numerator, denominator);
}
unsigned __aeabi_uidiv(unsigned numerator, unsigned denominator)
{
	return rom_div_helpers->rom_uidiv(numerator, denominator);
}

/* Modulo (%) */
void __aeabi_idivmod(int numerator, int denominator)
{
	struct idiv_return result = rom_div_helpers->rom_sidivmod(numerator, denominator);
	register uint32_t r0 asm("r0") = result.quotient;
	register uint32_t r1 asm("r1") = result.remainder;
	asm volatile("" : "+r"(r0), "+r"(r1) : "r"(r0), "r"(r1));
}
void __aeabi_uidivmod(unsigned numerator, unsigned denominator)
{
	struct uidiv_return result = rom_div_helpers->rom_uidivmod(numerator, denominator);
	register uint32_t r0 asm("r0") = result.quotient;
	register uint32_t r1 asm("r1") = result.remainder;
	asm volatile("" : "+r"(r0), "+r"(r1) : "r"(r0), "r"(r1));
}


/*******************************************************************************/
/*            In Application Programming ROM based routines                    */
/*******************************************************************************/

enum iap_commands {
	IAP_CMD_PREPARE_SECTORS_FOR_WRITE = 50,
	IAP_CMD_COPY_RAM_TO_FLASH = 51,
	IAP_CMD_ERASE_SECTORS = 52,
	IAP_CMD_BLANK_CHECK_SECTORS = 53,
	IAP_CMD_READ_PART_ID = 54,
	IAP_CMD_READ_BOOT_CODE_VERSION = 55,
	IAP_CMD_COMPARE = 56,
	IAP_CMD_REINVOQUE_ISP = 57,
	IAP_CMD_READ_UID = 58,
	IAP_CMD_ERASE_PAGE = 59,
	IAP_CMD_ERASE_INFO_PAGE = 60,
};

typedef void (*iap_entry_func)(uint32_t*, uint32_t*);
iap_entry_func iap_entry;

static uint32_t params[5];
static uint32_t results[4];

int iap_erase_info_page(uint32_t start_page, uint32_t end_page)
{
	params[0] = IAP_CMD_ERASE_INFO_PAGE;
	params[1] = start_page;
	params[2] = end_page;
	params[3] = (get_main_clock() / 1000);
	lpc_disable_irq();
	iap_entry(params, results);
	lpc_enable_irq();
	return results[0];
}

int iap_prepare_flash(uint32_t start_sector, uint32_t end_sector)
{
	params[0] = IAP_CMD_PREPARE_SECTORS_FOR_WRITE;
	params[1] = start_sector;
	params[2] = end_sector;
	iap_entry(params, results);
	return results[0];
}

int iap_erase_flash_sectors(uint32_t start_sector, uint32_t end_sector)
{
	params[0] = IAP_CMD_ERASE_SECTORS;
	params[1] = start_sector;
	params[2] = end_sector;
	params[3] = (get_main_clock() / 1000);
	lpc_disable_irq();
	iap_entry(params, results);
	lpc_enable_irq();
	return results[0];
}

int iap_erase_flash_pages(uint32_t start_page, uint32_t end_page)
{
	params[0] = IAP_CMD_ERASE_PAGE;
	params[1] = start_page;
	params[2] = end_page;
	params[3] = (get_main_clock() / 1000);
	lpc_disable_irq();
	iap_entry(params, results);
	lpc_enable_irq();
	return results[0];
}
int iap_copy_ram_to_flash(uint32_t dest, uint32_t src, uint32_t size)
{
	params[0] = IAP_CMD_COPY_RAM_TO_FLASH;
	params[1] = dest & ~(0x03);
	params[2] = src & ~(0x03);
	params[3] = size & ~(0x03);
	params[4] = (get_main_clock() / 1000);
	lpc_disable_irq();
	iap_entry(params, results);
	lpc_enable_irq();
	return results[0];
}

uint32_t iap_read_part_id(void)
{
	params[0] = IAP_CMD_READ_PART_ID;
	iap_entry(params, results);
	return results[1];
}


/*******************************************************************************/
/*            Rom based routines initialisation                                */
/*******************************************************************************/
#define LPC_122x_DIV_ROM_LOC (0x1FFC0000)
#define LPC_122x_IAP_ROM_LOC (0x1FFF1FF1)

void rom_helpers_init(void)
{
	rom_div_helpers = *((struct lpc_rom_div_helpers**)LPC_122x_DIV_ROM_LOC);
	iap_entry = (iap_entry_func)LPC_122x_IAP_ROM_LOC;
}

