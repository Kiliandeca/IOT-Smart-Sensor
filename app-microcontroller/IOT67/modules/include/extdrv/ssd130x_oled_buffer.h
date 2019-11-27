/****************************************************************************
 *   extdrv/ssd130x_oled_buffer.h
 *
 * Set of functions helping using the 128x64 buffer for the SSD130X
 *
 * Copyright 2017 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef EXTDRV_SSD130X_OLED_BUFFER_H
#define EXTDRV_SSD130X_OLED_BUFFER_H

#include "lib/stdint.h"

/* Set whole display to given value */
int ssd130x_buffer_set(uint8_t* gddram, uint8_t val);

/* Change our internal buffer, without actually displaying the changes */
int ssd130x_buffer_set_pixel(uint8_t* gddram, uint8_t x0, uint8_t y0, uint8_t state);
int ssd130x_buffer_set_tile(uint8_t* gddram, uint8_t x0, uint8_t y0, uint8_t* tile);

/* Simple RLE decompressor (two implementations, wip) */
void uncompress_image(const uint8_t* compressed_data,
                      uint8_t* buffer);
void uncompress_image_asm(const uint8_t* compressed_data,
                          uint8_t* buffer);
void uncompress_image_asm_old(const uint8_t* compressed_data,
                          uint8_t* buffer);

#endif
