/****************************************************************************
 *   core/system.h
 *
 * All low-level functions for clocks configuration and switch, system
 *  power-up, reset, and power-down.
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

#ifndef CORE_SYSTEM_H
#define CORE_SYSTEM_H

#include "lib/stdint.h"

#include "core/lpc_regs.h"
#include "core/lpc_core.h"
#include "core/watchdog.h"


/***************************************************************************** */
/*                       Power up defaults                                     */
/***************************************************************************** */
/* Change reset power state to our default, removing power from unused interfaces */
void system_set_default_power_state(void);


/***************************************************************************** */
/*                       Power                                                 */
/***************************************************************************** */
/* Enter deep sleep.
 * NOTE : entering deep sleep implies a lot of side effects. I'll try to list them all here
 *        so this can be done right.
 *
 * Note : see remark about RTC and deep sleep in section 5.3.3 of UM10441
 */
void enter_deep_sleep(void);

/* Enter deep power down.
 * NOTE : entering deep power down implies a lot of side effects. I'll try to list them all here
 *        so this can be done right.
 *    - The device watchdog must not have deep power-down locked.
 *
 * There are only two ways to get out of deep power down : RTC interrupt and wakeup pin going low.
 *   Even RESET pin won't get the chip out of power down.
 */
void enter_deep_power_down(void);


/* Power on or off a subsystem */
void subsystem_power(uint32_t power_bit, uint32_t on_off);
/* Check whether a subsystem is powered or not */
uint8_t subsystem_powered(uint32_t power_bit);


/* Configure the brown-out detection.
 * Note: Brown-Out detection must be powered to operate the ADC (See Section 19.2
 *    of UM10441 revision 2.1 or newer for more information)
 */
void system_brown_out_detection_config(uint32_t level);


/***************************************************************************** */
/*                      System Clock                                           */
/***************************************************************************** */
/* A clock frequency is defined as the integer value in MHz divided by 12, shifted
 * by 3 and or'ed with the value to be programmed in the flash config register for
 * the flash access time at the given frequency shifted by one and the flash
 * override bit in the LSB
 */
/* PLL may fail to lock for frenquencies above 60MHz */
#define FREQ_SEL_60MHz   ((5 << 3) | (0x01 << 1) | 0)
#define FREQ_SEL_48MHz   ((4 << 3) | (0x00 << 1) | 0)
#define FREQ_SEL_36MHz   ((3 << 3) | (0x00 << 1) | 0)
#define FREQ_SEL_24MHz   ((2 << 3) | (0x00 << 1) | 1)
#define FREQ_SEL_12MHz   ((1 << 3) | (0x00 << 1) | 1)
#define FREQ_SEL_IRC  FREQ_SEL_12MHz

/* Main clock config
 * We use internal RC and PLL0
 * Note that during PLL lock wait we are running on internal RC
 */
void clock_config(uint32_t freq_sel);

/* return current main clock in HZ */
uint32_t get_main_clock(void);


/* This is mainly a debug feature, but can be used to provide a clock to an
 * external peripheral */
void clkout_on(uint32_t src, uint32_t div);
void clkout_off(void);


/***************************************************************************** */
/* Sleeping functions : these use systick if the systick code is kept. Otherwise
 *   it will use a decrementing while loop which is (badly) calibrated for a 24MHz
 *   main clock.
 */
void msleep(uint32_t ms);
void usleep(uint32_t us);




/***************************************************************************** */
/*                     System Configuration                                    */
/***************************************************************************** */
/* System Configuration (SYSCON) */
struct lpc_sys_start_logic_ctrl
{
	volatile uint32_t edge_ctrl;  /* 0x00 : edge control Register 0 (R/W) */
	volatile uint32_t signal_en;  /* 0x04 : signal enable Register 0 (R/W) */
	volatile uint32_t reset;      /* 0x08 : reset Register 0  (-/W) */
	volatile uint32_t status;     /* 0x0C : status Register 0 (R/-) */
};
struct lpc_sys_config
{
	volatile uint32_t sys_mem_remap;   /* 0x000 System memory remap (R/W) */
	volatile uint32_t peripheral_reset_ctrl; /* 0x004 Peripheral reset control (R/W) */
	volatile uint32_t sys_pll_ctrl;    /* 0x008 System PLL control (R/W) */
	volatile uint32_t sys_pll_status;  /* 0x00C System PLL status (R/ ) */
	uint32_t reserved_0[4];

	volatile uint32_t sys_osc_ctrl;    /* 0x020 : System oscillator control (R/W) */
	volatile uint32_t WDT_osc_ctrl;    /* 0x024 : Watchdog oscillator control (R/W) */
	volatile uint32_t IRC_ctrl;        /* 0x028 : IRC control (R/W) */
	uint32_t reserved_1[1];
	volatile uint32_t sys_reset_status;    /* 0x030 : System reset status Register (R/ ) */
	uint32_t reserved_2[3];
	volatile uint32_t sys_pll_clk_sel;     /* 0x040 : System PLL clock source select (R/W) */
	volatile uint32_t sys_pll_clk_upd_en;  /* 0x044 : System PLL clock source update enable (R/W) */
	uint32_t reserved_3[10];

	volatile uint32_t main_clk_sel;     /* 0x070 : Main clock source select (R/W) */
	volatile uint32_t main_clk_upd_en;  /* 0x074 : Main clock source update enable (R/W) */
	volatile uint32_t sys_AHB_clk_div;  /* 0x078 : System AHB clock divider (R/W) */
	uint32_t reserved_4[1];

	volatile uint32_t sys_AHB_clk_ctrl; /* 0x080 : System AHB clock control (R/W) */
	uint32_t reserved_5[4];

	volatile uint32_t ssp0_clk_div;   /* 0x094 : SSP0 clock divider (R/W) */
	volatile uint32_t uart_clk_div[2];  /* 0x098 - 0x09C : UART0 and UART1 clock divider (R/W) */
	volatile uint32_t rtc_clk_div;    /* 0x0A0 : RTC clock divider (R/W) */
	uint32_t reserved_6[15];

	volatile uint32_t clk_out_src_sel; /* 0x0E0 : CLKOUT clock source select (R/W) */
	volatile uint32_t clk_out_upd_en;  /* 0x0E4 : CLKOUT clock source update enable (R/W) */
	volatile uint32_t clk_out_div;     /* 0x0E8 : CLKOUT clock divider (R/W) */
	uint32_t reserved_7[5];

	volatile uint32_t por_captured_io_status_0;  /* 0x100 : POR captured PIO status 0 (R/ ) */
	volatile uint32_t por_captured_io_status_1;  /* 0x104 : POR captured PIO status 1 (R/ ) */
	uint32_t reserved_8[11];

	volatile uint32_t IO_config_clk_div[7]; /* 0x134 - 0x14C : Peripheral clocks 6 to 0 for glitch filter */

	volatile uint32_t BOD_ctrl;      /* 0x150 : BOD control (R/W) */
	volatile uint32_t sys_tick_cal;  /* 0x154 : System tick counter calibration (R/W) */
	volatile uint32_t ahb_prio_set;  /* 0x158 : AHB priority setting (-/-) */
	uint32_t reserved_9[5];
	volatile uint32_t irq_latency;   /* 0x170 : IRQ delay, alloxs trade-off bw latency and determinism */
	volatile uint32_t int_nmi_cfg;   /* 0x174 : NMI interrupt source configuration control */
	uint32_t reserved_10[34];

	struct lpc_sys_start_logic_ctrl start_log_ctrl[2]; /* 0x200 to 0x20C and 0x210 to 0x21C :
												 Start logic 0 and Start logic 1/peripheral interrupts */
	uint32_t reserved_11[4];

	volatile uint32_t powerdown_sleep_cfg;  /* 0x230 : Power-down states in Deep-sleep mode (R/W) */
	volatile uint32_t powerdown_wake_cfg;  /* 0x234 : Power-down states after wake-up (R/W) */
	volatile uint32_t powerdown_run_cfg;        /* 0x238 : Power-down configuration Register (R/W) */
	uint32_t reserved_12[110];
	volatile const uint32_t device_id;  /* 0x3F4 : Device ID (R/ ) */
};

#define LPC_SYS_CONFIG ((struct lpc_sys_config *) LPC_SYSCON_BASE)

/* AHB control bits
 *   0 (System (cortexM0, syscon, PMU, ...)) is a read only bit (system cannot be disabled)
 */
#define LPC_SYS_ABH_CLK_CTRL_SYSTEM     (1 <<  0) /* Read only */
#define LPC_SYS_ABH_CLK_CTRL_ROM        (1 <<  1)
#define LPC_SYS_ABH_CLK_CTRL_RAM        (1 <<  2)
#define LPC_SYS_ABH_CLK_CTRL_FLASH_REG  (1 <<  3)
#define LPC_SYS_ABH_CLK_CTRL_FLASH      (1 <<  4)
#define LPC_SYS_ABH_CLK_CTRL_I2C        (1 <<  5)
#define LPC_SYS_ABH_CLK_CTRL_CRC        (1 <<  6)
#define LPC_SYS_ABH_CLK_CTRL_CT16B0     (1 <<  7)
#define LPC_SYS_ABH_CLK_CTRL_CT16B1     (1 <<  8)
#define LPC_SYS_ABH_CLK_CTRL_CT32B0     (1 <<  9)
#define LPC_SYS_ABH_CLK_CTRL_CT32B1     (1 << 10)
#define LPC_SYS_ABH_CLK_CTRL_SSP0       (1 << 11)
#define LPC_SYS_ABH_CLK_CTRL_UART0      (1 << 12)
#define LPC_SYS_ABH_CLK_CTRL_UART1      (1 << 13)
#define LPC_SYS_ABH_CLK_CTRL_ADC        (1 << 14)
#define LPC_SYS_ABH_CLK_CTRL_Watchdog   (1 << 15)
#define LPC_SYS_ABH_CLK_CTRL_IO_CONFIG  (1 << 16)
#define LPC_SYS_ABH_CLK_CTRL_DMA        (1 << 17)
#define LPC_SYS_ABH_CLK_CTRL_RTC        (1 << 19)
#define LPC_SYS_ABH_CLK_CTRL_CMP        (1 << 20)
#define LPC_SYS_ABH_CLK_CTRL_GPIO2      (1 << 29)
#define LPC_SYS_ABH_CLK_CTRL_GPIO1      (1 << 30)
#define LPC_SYS_ABH_CLK_CTRL_GPIO0      (1 << 31)
/* Helper */
#define LPC_SYS_ABH_CLK_CTRL_MEM_ALL    0x0000001F
/* Flash override in Peripheral reset control register */
#define LPC_FLASH_OVERRIDE  (1 << 15 )

#define LPC_SSP_RESET_N        (1 << 0)
#define LPC_I2C_RESET_N        (1 << 1)
#define LPC_UART0_RESET_N      (1 << 2)
#define LPC_UART1_RESET_N      (1 << 3)

#define LPC_POWER_DOWN_IRC_OUT      (1 << 0)
#define LPC_POWER_DOWN_IRC          (1 << 1)
#define LPC_POWER_DOWN_FLASH        (1 << 2)
#define LPC_POWER_DOWN_BOD          (1 << 3)
#define LPC_POWER_DOWN_ADC          (1 << 4)
#define LPC_POWER_DOWN_SYS_OSC      (1 << 5)
#define LPC_POWER_DOWN_WDT_OSC      (1 << 6)
#define LPC_POWER_DOWN_SYSPLL       (1 << 7)
#define LPC_POWER_DOWN_COPARATOR    (1 << 15)

#define LPC_DEEP_SLEEP_CFG_NOWDTLOCK_BOD_ON  0x0000FFF7
#define LPC_DEEP_SLEEP_CFG_NOWDTLOCK_BOD_OFF 0x0000FFFF

#define LPC_MAIN_CLK_SRC_IRC_OSC       0x00
#define LPC_MAIN_CLK_SRC_PLL_IN        0x01
#define LPC_MAIN_CLK_SRC_WATCHDOG_OSC  0x02
#define LPC_MAIN_CLK_SRC_PLL_OUT       0x03

#define LPC_PLL_CLK_SRC_IRC_OSC        0x00
#define LPC_PLL_CLK_SRC_EXT_OSC        0x01

#define LPC_CLKOUT_SRC_IRC_OSC       0x00
#define LPC_CLKOUT_SRC_XTAL_OSC      0x01
#define LPC_CLKOUT_SRC_WATCHDOG_OSC  0x02
#define LPC_CLKOUT_SRC_MAIN_CLK      0x03

#define LPC_WDT_DIVSEL(x)  (((x) / 2) - 1)
#define LPC_WDT_FREQSEL_600KHz  (0x01 << 5)

/***************************************************************************** */
/*                  Flash Control                                              */
/***************************************************************************** */
/* Flash configuration */
struct lpc_flash_control
{
	uint32_t reserved_0[10];
	volatile uint32_t flash_cfg; /* 0x028 Flash configuration (R/W) */
};
#define LPC_FLASH_CONTROL ((struct lpc_flash_control *) LPC_FLASH_CONFIG_BASE)
#define LPC_FLASH_CFG_MASK   0x03
#define LPC_FLASH_CFG_SHIFT  0


/***************************************************************************** */
/*                     Power Management Unit                                   */
/***************************************************************************** */
/* Power Management Unit (PMU) */
struct lpc_pm_unit
{
	volatile uint32_t power_ctrl;  /* 0x000 : Power control Register (R/W) */
	volatile uint32_t gp_data[4];  /* 0x004 to 0x010 : General purpose Register 0 to 3 (R/W) */
	volatile uint32_t system_config; /* 0x014 : System configuration register (R/W) */
							/* (RTC clock control and hysteresis of the WAKEUP pin) */
};
#define LPC_PMU         ((struct lpc_pm_unit *) LPC_PMU_BASE)

/* Power control register */
#define LPC_PD_EN                   (0x01 << 1)
#define LPC_SLEEP_FLAG              (0x01 << 8)
#define LPC_DPD_FLAG                (0x01 << 11)
/* System config register */
#define LPC_WAKEUP_PIN_HYST_MASK    (0x01 << 10)
#define LPC_RTC_CLK_SRC_SHIFT       11
#define LPC_RTC_CLK_SRC_MASK        (0x0F << LPC_RTC_CLK_SRC_SHIFT)
/* See RTC section above for RTC Clock source selection bits */


#endif /* CORE_SYSTEM_H */
