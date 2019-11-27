/****************************************************************************
 *   extdrv/eeprom.h
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

#ifndef EXTDRV_EEPROM_H
#define EXTDRV_EEPROM_H

#include "lib/stdint.h"


/* FIXME : All of this will have to be re-written with an eeprom definition structure */

/***************************************************************************** */
/*          Read and Write for system eeprom                                   */
/***************************************************************************** */
/* EEPROM Read
 * Performs a non-blocking read on the eeprom.
 *   eeprom_addr : address of the I2C eeprom chip (hardware dependent)
 *   offset : data offset in eeprom.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes read. On error, returns a negative
 *   integer equivalent to errors from glibc.
 */
int eeprom_read(uint8_t eeprom_addr, uint32_t offset, void *buf, size_t count);

/* EEPROM Write
 * Performs a non-blocking write on the eeprom.
 *   eeprom_addr : address of the I2C eeprom chip (hardware dependent)
 *   offset : data offset in eeprom.
 * RETURN VALUE
 *   Upon successfull completion, returns the number of bytes written. On error, returns a negative
 *   integer equivalent to errors from glibc.
 */
int eeprom_write(uint8_t eeprom_addr, uint32_t offset, const void *buf, size_t count);




#endif /* EXTDRV_EEPROM_H */
