/****************************************************************************
 *   core/lpc_regs.h
 *
 * LPC122x Cortex-M0 Core Registers definitions
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
#ifndef LPC_REGS_H
#define LPC_REGS_H

#include "lib/stddef.h"
#include "lib/stdint.h"



/***************************************************************************** */
/*                          Memory Map                                         */
/***************************************************************************** */
/* Base addresses */
#define LPC_FLASH_BASE        (0x00000000UL)
#define LPC_RAM_BASE          (0x10000000UL)
#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_APB1_BASE         (0x40080000UL) /* unused in LPC12xx */
#define LPC_AHB_BASE          (0x50000000UL)

/* Memory mapping of Cortex-M0 Hardware */
#define LPC_SCS_BASE        (0xE000E000UL)         /* System Control Space Base Address */
#define LPC_COREDEBUG_BASE  (0xE000EDF0UL)         /* Core Debug Base Address */
#define LPC_SYSTICK_BASE    (LPC_SCS_BASE + 0x0010UL)  /* SysTick Base Address */
#define LPC_NVIC_BASE       (LPC_SCS_BASE + 0x0100UL)  /* NVIC Base Address */
#define LPC_SCB_BASE        (LPC_SCS_BASE + 0x0D00UL)  /* System Control Block Base Address */

/* APB0 peripherals */
#define LPC_I2C0_BASE          (LPC_APB0_BASE + 0x00000)
#define LPC_WDT_BASE           (LPC_APB0_BASE + 0x04000)
#define LPC_UART0_BASE         (LPC_APB0_BASE + 0x08000)
#define LPC_UART1_BASE         (LPC_APB0_BASE + 0x0C000)
#define LPC_TIMER0_BASE        (LPC_APB0_BASE + 0x10000)
#define LPC_TIMER1_BASE        (LPC_APB0_BASE + 0x14000)
#define LPC_TIMER2_BASE        (LPC_APB0_BASE + 0x18000)
#define LPC_TIMER3_BASE        (LPC_APB0_BASE + 0x1C000)
#define LPC_ADC_BASE           (LPC_APB0_BASE + 0x20000)
#define LPC_PMU_BASE           (LPC_APB0_BASE + 0x38000)
#define LPC_SSP0_BASE          (LPC_APB0_BASE + 0x40000)
#define LPC_IOCON_BASE         (LPC_APB0_BASE + 0x44000)
#define LPC_SYSCON_BASE        (LPC_APB0_BASE + 0x48000)
#define LPC_DMA_BASE           (LPC_APB0_BASE + 0x4C000)
#define LPC_RTC_BASE           (LPC_APB0_BASE + 0x50000)
#define LPC_COMPARATOR_BASE    (LPC_APB0_BASE + 0x54000)

/* AHB peripherals */
#define LPC_GPIO_0_BASE        (LPC_AHB_BASE + 0x00000)
#define LPC_GPIO_1_BASE        (LPC_AHB_BASE + 0x10000)
#define LPC_GPIO_2_BASE        (LPC_AHB_BASE + 0x20000)
#define LPC_FLASH_CONFIG_BASE  (LPC_AHB_BASE + 0x60000)
#define LPC_CRC_BASE           (LPC_AHB_BASE + 0x70000)


/* User information block */
#define LPC_START_INFO_PAGES   (0x00040200)
#define LPC_END_INFO_PAGES     (LPC_START_INFO_PAGES + 0x600)



#endif  /* LPC_REGS_H */
