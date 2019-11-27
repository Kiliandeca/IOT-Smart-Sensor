/****************************************************************************
 *  core/crc_engine.h
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


#ifndef CORE_CRC_ENGINE_H
#define CORE_CRC_ENGINE_H

/***************************************************************************** */
/*                CRC engine                                                   */
/***************************************************************************** */

/* CRC engine configuration and utility functions
 * Refer to the LPC122x documentation (UM10441.pdf) for more information
 */


#include "lib/stdint.h"
#include "core/lpc_regs.h"



/***************************************************************************** */
/* Configure the CRC engine. */
void crc_config(uint8_t poly, uint32_t seed, uint8_t data_mode, uint8_t sum_mode);


void crc_reinit(void);

uint32_t crc_get_sum(void);

void crc_add(uint32_t value);




/***************************************************************************** */
/*                  CRC engine                                                 */
/***************************************************************************** */
/* CRC engine */
struct lpc_crc_engine
{
	volatile uint32_t mode; /* 0x000 CRC Mode (R/W) */
	volatile uint32_t seed; /* 0x004 CRC Seed (R/W) */
	union {
		volatile uint32_t sum;  /* 0x008 CRC CRC Checksum (R/-) */
		volatile uint32_t data; /* 0x008 CRC Data (-/W) */
	};
};
#define LPC_CRC_ENGINE  ((struct lpc_crc_engine*)LPC_CRC_BASE)

/* Mode register */
#define LPC_CRC_POLY_CRC32   (0x02)
#define LPC_CRC_POLY_CRC16   (0x01)
#define LPC_CRC_POLY_CRCCITT (0x00)
#define LPC_CRC_REVERSE_DATA_BIT_ORDER (0x01 << 2)
#define LPC_CRC_DATA_COMPLEMENT        (0x01 << 3)
#define LPC_CRC_REVERSE_SUM_BIT_ORDER  (0x01 << 4)
#define LPC_CRC_SUM_COMPLEMENT         (0x01 << 5)



#endif  /* CORE_CRC_ENGINE_H */


