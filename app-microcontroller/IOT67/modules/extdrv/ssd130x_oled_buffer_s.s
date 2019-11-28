@   extdrv/ssd130x_oled_buffer.c
@
@ Set of functions helping using the 128x64 buffer for the SSD130X
@
@ Copyright 2017 Nathael Pajani <nathael.pajani@ed3l.fr>
@
@
@ This program is free software: you can redistribute it and/or modify
@ it under the terms of the GNU General Public License as published by
@ the Free Software Foundation, either version 2 of the License, or
@ (at your option) any later version.
@
@ This program is distributed in the hope that it will be useful,
@ but WITHOUT ANY WARRANTY; without even the implied warranty of
@ MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@ GNU General Public License for more details.
@
@ You should have received a copy of the GNU General Public License
@ along with this program.  If not, see <http://www.gnu.org/licenses/>.

.syntax unified
.cpu cortex-m0
.thumb

@ exported symbol:
.global uncompress_image_asm

/* Simple RLE decompressor, asm implementation, experimental */
.thumb_func
uncompress_image_asm:
 push    {r4,r5,r14}  @
 movs    r2, 128      @
 lsls    r2, 3        @
 adds    r2, r2, r1   @
.label1:
 ldrb    r3, [r0]     @
 adds    r0, 1        @
 sxtb    r3, r3       @
 cmp     r3, 0        @
 bpl     .set
 rsbs    r3, r3, 0    @ negs...
 movs    r5, 1        @
 b       .copy2
.set:
 adds    r3, 3        @
 movs    r5, 0        @
.copy2:
 ldrb    r4, [r0]     @
 strb    r4, [r1]     @
 adds    r0, r5       @
 adds    r1, 1        @
 subs    r3, 1        @
 bne     .copy2
 subs    r0, r0, r5   @
 adds    r0, r0, 1    @
 cmp     r1, r2       @
 bne     .label1
 pop     {r4,r5,r15}  @ restaure r4 & r5
