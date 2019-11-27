/****************************************************************************
 *  lib/crc_ccitt.h
 *
 * Copyright 2017 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#include "lib/stdint.h"

/***************************************************************************** */
/* CRC CCITT computation, using the above static lookup table for performance.
 * If called for the first time for a given set of chunks of data, provide the
 *   start value (usually 0x0000, 0xFFFF or 0x1D0F) as crc parameter.
 * Provide the previous crc value returned for each successive call corresponding to
 *   a new chunk of data in a given set.
 * buf is a pointer to the buffer holding the data, and len the length of the data in
 *   the buffer.
 * Returns the updated crc if buf is not null, or the old value when buff is null.
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t* buf, int len);

