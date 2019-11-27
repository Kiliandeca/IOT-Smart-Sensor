/****************************************************************************
 *   extdrv/ssd130x_oled_buffer.c
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

#include "lib/string.h"

#include "extdrv/ssd130x_oled_driver.h"

/* Set whole display to given value */
int ssd130x_buffer_set(uint8_t *gddram, uint8_t val)
{
	memset(gddram + 4, val, GDDRAM_SIZE);
	return 0;
}

/* Change our internal buffer, without actually displaying the changes */
int ssd130x_buffer_set_pixel(uint8_t* gddram, uint8_t x0, uint8_t y0, uint8_t state)
{
	uint8_t* addr = gddram + 4 + ((y0 / 8) * 128) + x0;
	if (state != 0) {
		*addr |=  (0x01 << (y0 % 8));
	} else {
		*addr &= ~(0x01 << (y0 % 8));
	}
	return 0;
}

/* Change a "tile" in the bitmap memory.
 * A tile is a 8x8 pixels region, aligned on a 8x8 grid representation of the display.
 *  x0 and y0 are in number of tiles.
 */
int ssd130x_buffer_set_tile(uint8_t* gddram, uint8_t x0, uint8_t y0, uint8_t* tile)
{
	uint8_t* addr = gddram + 4 + (y0 * 128) + (x0 * 8);
	memcpy(addr, tile, 8);
	return 0;
}

/* Simple RLE decompressor */
void uncompress_image(const uint8_t *compressed_data,
                      uint8_t *buffer)
{
	int i;
	signed char *in	= (signed char *)compressed_data;
	uint8_t* out = buffer;
	do {
		if (*in < 0) {
			int count = -(int)*in;
			in++;
			for (i = 0; i < count; i++)
				*out++ = *in++;
		} else {
			int count = *in + 3;
			in++;
			for (i = 0; i < count; i++)
				*out++ = *in;
			in++;
		}
	} while (out - buffer < 8 * 128);
}

/* Simple RLE decompressor, asm implementation, experimental */
void uncompress_image_asm_old(const uint8_t *compressed_data,
                              uint8_t *buffer)
{
  // note: r0 is compressed_data/in, r1 is buffer
  // r2 is end_buffer
  // r3 is count
  // r4 is data
  // r5 is in-offset
  asm volatile(
               ".syntax unified\n\t"
               "push    {r2,r3,r4,r5}\n\t"
               "movs    r2, 128\n\t"
               "lsls    r2, 3\n\t"
               "adds    r2, r2, r1\n\t"
               ".label1:\n\t"
               "  ldrb    r3, [r0]\n\t"
               "  adds    r0, 1\n\t"
               "  sxtb    r3, r3\n\t"
               "  cmp     r3, 0\n\t"
               "  bpl     .set\n\t"
               "    rsbs    r3, r3, 0\t@ negs...\n\t"
               "    movs    r5, 1\n\t"
               "    b       .copy2\n"
               ".set:\n\t"
               "    adds    r3, 3\n\t"
               "    movs    r5, 0\n"
               ".copy2:\n\t"
               "    ldrb    r4, [r0]\n\t"
               "    strb    r4, [r1]\n\t"
               "    adds    r0, r5\n\t"
               "    adds    r1, 1\n\t"
               "    subs    r3, 1\n\t"
               "    bne     .copy2\n\t"
               "    subs    r0, r0, r5\n\t"
               "    adds    r0, r0, 1\n\t"
               "  cmp     r1, r2\n\t"
               "  bne     .label1\n"
               ".end:\n\t"
               "pop     {r2,r3,r4,r5}\n\t"
              :
              );
}
