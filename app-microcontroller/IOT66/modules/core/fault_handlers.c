/****************************************************************************
 *   core/fault_handlers.c
 *
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 * Example code from frozeneskimo.com :
 *   http://dev.frozeneskimo.com/notes/getting_started_with_cortex_m3_cmsis_and_gnu_tools
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

/* Default "fault" handlers, which catch the fault exceptions.
 * These are defined as weak aliases of a dummy fault handler which enters an empty infinite
 *   loop and chould be overidden by user defined handlers.
 */

#include "lib/stdint.h"

void fault_info(const char* name, uint32_t len) __attribute__ ((weak, alias ("Dummy_Fault_Handler")));

void Dummy_Fault_Handler(const char* name, uint32_t len) {
	while (1);
}

/* Cortex M0 core interrupt handlers */
void NMI_Handler(void)
{
	fault_info(__FUNCTION__, sizeof(__FUNCTION__));
}
void HardFault_Handler(void)
{
	fault_info(__FUNCTION__, sizeof(__FUNCTION__));
}
void SVC_Handler(void)
{
	fault_info(__FUNCTION__, sizeof(__FUNCTION__));
}
void PendSV_Handler(void)
{
	fault_info(__FUNCTION__, sizeof(__FUNCTION__));
}



