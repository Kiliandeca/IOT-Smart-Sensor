/****************************************************************************
 *  core/crc_engine
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

/***************************************************************************** */
/*                CRC engine                                                   */
/***************************************************************************** */

/* CRC engine configuration and utility functions
 * Refer to the LPC122x documentation (UM10441.pdf) for more information
 */


#include "lib/stdint.h"
#include "core/crc_engine.h"


static uint32_t crc_seed = 0;

/***************************************************************************** */
/* Configure the CRC engine. */
void crc_config(uint8_t poly, uint32_t seed, uint8_t data_mode, uint8_t sum_mode)
{
	struct lpc_crc_engine* crc_ctrl = LPC_CRC_ENGINE;

	crc_ctrl->mode = ((poly & 0x03) | (data_mode & 0x0C) | (sum_mode & 0x30));
	crc_ctrl->seed = seed;
	crc_seed = seed;
}


void crc_reinit(void)
{
	struct lpc_crc_engine* crc_ctrl = LPC_CRC_ENGINE;
	crc_ctrl->seed = crc_seed;
}

uint32_t crc_get_sum(void)
{
	struct lpc_crc_engine* crc_ctrl = LPC_CRC_ENGINE;
	return crc_ctrl->sum;
}


void crc_add(uint32_t value)
{
	struct lpc_crc_engine* crc_ctrl = LPC_CRC_ENGINE;
	crc_ctrl->sum = value;
}



