/****************************************************************************
 *   core/vector_table.c
 *
 *
 * Copyright 2015 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 * This file is derived from Work covered by the Apache 2.0 licence.
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Original sources should be found there :
 *     https://github.com/mbedmicro/CMSIS-DAP.git
 * Original file name is bootloader/hal/TARGET_NXP/TARGET_LPC11U35/vector_table.c
 *
 * Original copyright notice :
 *    CMSIS-DAP Interface Firmware
 *    Copyright (c) 2009-2013 ARM Limited
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
 *****************************************************************************/

/* This file holds the code related to the vector table relocation in RAM.
 * This mechanism is used by the IAP code before re-programming the first sector of the internal
 *   flash memory which usually holds the active vector table.
 */

#include "lib/stdint.h"
#include "core/system.h"
#include "core/iap.h"

#define NVIC_NUM_VECTORS (16 + 32)            /* CORE + MCU Peripherals */
#define NVIC_RAM_VECTOR_ADDRESS (0x10000000)  /* Vectors positioned at start of RAM */

void relocate_vector_table(void)
{
	struct lpc_sys_config* sysctrl = LPC_SYS_CONFIG;
    int i;
    /* Space for dynamic vectors, initialised to allocate in R/W */
    static volatile uint32_t * vectors = (uint32_t*)NVIC_RAM_VECTOR_ADDRESS;

    /* Copy and switch to dynamic vectors if first time called */
    if((sysctrl->sys_mem_remap & 0x3) != 0x1) {
        uint32_t *old_vectors = (uint32_t *)START_APP_ADDRESS;
        for(i = 0; i < NVIC_NUM_VECTORS; i++) {
            vectors[i] = old_vectors[i];
        }
		sysctrl->sys_mem_remap = 0x1;
    }
}
