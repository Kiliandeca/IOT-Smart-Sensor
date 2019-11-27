/****************************************************************************
 *  lib/stdlib.h
 *
 * Copyright 2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef LIB_STDLIB_H
#define LIB_STDLIB_H

#include "lib/stdint.h"

/* Simple strtoul implementation.
 * Returns the value converted from the given string.
 * Does not check that the base is respected aside for the use of letters in
 *   number representation.
 */
uint32_t strtoul(const char* str, char** end, uint8_t base);

#endif /* LIB_STDLIB_H */

