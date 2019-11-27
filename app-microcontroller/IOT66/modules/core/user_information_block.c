/****************************************************************************
 *   core/user_information_block.c
 *
 *
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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


/* The user information block is persistant flash memory block which can be programmed
 *   and read by user code using IAP calls.
 * It can provide the same functionnality as an EEPROM, though the reading access is
 *   much more simple.
 * Writting to the user flash is slightly more complicated though, mainly because flash
 *   pages in the user information block are big 512 bytes blocks, much more than the
 *   usual small eeprom pages.
 * One must erase a full page before modifying data in it, thus reading it first to RAM,
 *   modifying the RAM area, erasing the page, and then writting back.
 * Reading/modify/write may be limited to the required size, but erase size will always
 *   be a full page.
 */

/* The user information block of the LPC122x has three 512 bytes pages. */

/* Actually, the only purpose of this code is to return the address of the begining
 *   of the user information block in flash, set by the linker from information in
 *   the liker script (lpc_link_lpc1224.ld).
*/

#include "lib/stdint.h"
#include "core/lpc_regs.h"


/* Get a pointer to the user information */
void* get_user_info(void)
{
	return (void*)LPC_START_INFO_PAGES;
}


