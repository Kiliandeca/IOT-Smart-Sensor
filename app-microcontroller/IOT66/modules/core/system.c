/****************************************************************************
 *   core/system.c
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

/*
 * This file holds some system wide initialisation functions and clock or sleep
 *   related functions.
 */

#include "core/system.h"


/* Private defines */
#define LPC_IRC_OSC_CLK  (12000000UL)  /* Internal RC oscillator frequency : 12MHz */

/***************************************************************************** */
/*                     Global data                                             */
/***************************************************************************** */
struct lpc_desc_private {
	uint32_t main_clock;
	uint8_t brown_out_detection_enabled;
	uint8_t need_IRC;
};
static struct lpc_desc_private lpc_private = {
	.main_clock = LPC_IRC_OSC_CLK,
	.brown_out_detection_enabled = 0,
	.need_IRC = 1,
};

/***************************************************************************** */
/*                     Required system inits                                  */
/***************************************************************************** */
/* Set up number of CPU clocks for flash access (see chapter 4.5.42 and 4.10.4 of
 *  LPC12xx User manual (UM10441.pdf))
 * freq_sel : code for CPU freq, as defined in system.h header file :
 * A clock frequency is defined as the integer value in MHz shifted by 3 and or'ed
 * with the value to be programmed in the flash config register for the flash access
 * time at the given frequency shifted by one and the flash override bit in the LSB
 */
static void flash_accelerator_config(uint32_t freq_sel)
{
	struct lpc_flash_control* fcfg = LPC_FLASH_CONTROL;
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;

	if (freq_sel & 0x01) {
		/* 1 cycle read mode */
		sys_config->peripheral_reset_ctrl |= LPC_FLASH_OVERRIDE;
	} else {
		/* multiple cycle flash read mode */
		sys_config->peripheral_reset_ctrl &= ~(LPC_FLASH_OVERRIDE);
		fcfg->flash_cfg &= ~(LPC_FLASH_CFG_MASK);
		fcfg->flash_cfg |= ((freq_sel & 0x06) >> 1);
	}
}

/* Configure the brown-out detection */
void system_brown_out_detection_config(uint32_t level)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;

	if (level == 0) {
		/* Disable Brown-Out Detection, power it down */
		sys_config->powerdown_run_cfg |= LPC_POWER_DOWN_BOD;
		lpc_private.brown_out_detection_enabled = 0;
	} else {
		/* Power on Brown-Out Detection.
		 * (Needed for ADC, See Section 19.2 of UM10441 revision 2.1 or newer for more information) */
		sys_config->powerdown_run_cfg &= ~(LPC_POWER_DOWN_BOD);
		lpc_private.brown_out_detection_enabled = 1;
		/* Configure Brown-Out Detection */
		/* FIXME */
	}
}

/***************************************************************************** */
/*                       Power                                                 */
/***************************************************************************** */
/* Set up power-down control
 * Change reset power state to our default, removing power from unused interfaces
 */
void system_set_default_power_state(void)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	/* Start with all memory powered on and nothing else */
	sys_config->sys_AHB_clk_ctrl = LPC_SYS_ABH_CLK_CTRL_MEM_ALL;
}

/* Enter deep sleep.
 * NOTE : entering deep sleep implies a lot of side effects. I'll try to list them all here
 *        so this can be done right.
 *
 * Note : see remark about RTC and deep sleep in section 5.3.3 of UM10441
 */
void enter_deep_sleep(void)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;

	/* Ask for the same clock status when waking up */
	sys_config->powerdown_wake_cfg = sys_config->powerdown_run_cfg;
	/* Set deep_sleep config */
	if (lpc_private.brown_out_detection_enabled) {
		sys_config->powerdown_sleep_cfg = LPC_DEEP_SLEEP_CFG_NOWDTLOCK_BOD_ON;
	} else {
		sys_config->powerdown_sleep_cfg = LPC_DEEP_SLEEP_CFG_NOWDTLOCK_BOD_OFF;
	}
	/* Enter deep sleep */
	/* FIXME */
}

/* Enter deep power down.
 * NOTE : entering deep power down implies a lot of side effects. I'll try to list them all here
 *        so this can be done right.
 *    - The device watchdog must not have deep power-down locked.
 *
 * Note : see remark about RTC and deep sleep in section 5.3.3 of UM10441
 */
void enter_deep_power_down(void)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct syst_ctrl_block_regs* scb = LPC_SCB;
	struct lpc_pm_unit* pmu = LPC_PMU;

	/* Clear deep power down flag */
	pmu->power_ctrl |= LPC_DPD_FLAG;

	/* Ask for the same clock status when waking up */
	sys_config->powerdown_wake_cfg = sys_config->powerdown_run_cfg;

	/* Set power-down sleep config : Turn Watchdog and BOD off when entering deep power down */
	sys_config->powerdown_sleep_cfg = 0xFFFF;
	/* Switch to low-speed WDO */
	sys_config->main_clk_sel = LPC_MAIN_CLK_SRC_IRC_OSC;
	sys_config->main_clk_upd_en = 0;
	sys_config->main_clk_upd_en = 1;
	while (!(sys_config->main_clk_upd_en & 0x01));

	/* Enter deep power-down on next wfi() call */
	pmu->power_ctrl |= LPC_PD_EN;
	scb->scr |= SCB_SCR_SLEEPDEEP;

	/* Power down everything but the IRC and Flash */
	sys_config->powerdown_run_cfg &= ~(LPC_POWER_DOWN_IRC_OUT | LPC_POWER_DOWN_IRC | LPC_POWER_DOWN_FLASH);

	/* "wfi" instruction to enter deep power-down */
	wfi();
}

/* Power on or off a subsystem */
void subsystem_power(uint32_t power_bit, uint32_t on_off)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	if (on_off == 1) {
		sys_config->sys_AHB_clk_ctrl |= power_bit;
	} else {
		sys_config->sys_AHB_clk_ctrl &= ~(power_bit);
	}
}
/* Check whether a subsystem is powered or not */
uint8_t subsystem_powered(uint32_t power_bit)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	return (sys_config->sys_AHB_clk_ctrl & power_bit);
}

/***************************************************************************** */
/*                      System Clock                                           */
/***************************************************************************** */
static void propagate_main_clock(void);

/* Main clock config : set up the system clock
 * We use internal RC oscilator as sys_pllclkin if pll is ussed
 * Note that during PLL lock wait we are running on internal RC
 * Note for M and P calculation :
 *          FCCO must be between 156MHz and 320MHz
 *          M ranges from 1 to 32, steps of 1, M = Freq_out / Freq_in 
 *          P ranges from 1 to 8, P must be a power of two, FCCO = 2 * P * Freq_out
 *          Freq_out < 100 MHz
 *          Freq_in is between 10 and 25 MHz
 *          M = Freq_out / Freq_in 
 *
 * freq_sel : set to one of the predefined values. See core/system.h
 */
void clock_config(uint32_t freq_sel)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;

	lpc_disable_irq();
	/* Turn on IRC */
	sys_config->powerdown_run_cfg &= ~(LPC_POWER_DOWN_IRC);
	sys_config->powerdown_run_cfg &= ~(LPC_POWER_DOWN_IRC_OUT);
	/* Use IRC clock for main clock */
	sys_config->main_clk_sel = LPC_MAIN_CLK_SRC_IRC_OSC;
	lpc_private.need_IRC = 1;
	/* Switch the main clock source */
	sys_config->main_clk_upd_en = 0;
	sys_config->main_clk_upd_en = 1;

	/* Turn off / power off external crystal */
	sys_config->powerdown_run_cfg |= LPC_POWER_DOWN_SYS_OSC;
	/* Set AHB clock divider : divide by one ... */
	sys_config->sys_AHB_clk_div = 1;
	/* Configure number of CPU clocks for flash access before setting the new clock */
	flash_accelerator_config(freq_sel);

	/* power off PLL */
	sys_config->powerdown_run_cfg |= LPC_POWER_DOWN_SYSPLL;

	/* If using only internal RC, we are done */
	if (freq_sel == FREQ_SEL_IRC) {
		lpc_private.main_clock = LPC_IRC_OSC_CLK;
	} else {
		uint32_t M = ((freq_sel >> 3) & 0xFF);
		uint32_t N = 0; /* P = 2 ^ N */

		/* Select value for N */
		switch (freq_sel) {
			case 3: /* FREQ_SEL_36MHz */
			case 2: /* FREQ_SEL_24MHz */
				N = 2; /* P = 4 */
				break;
			default: /* 48MHz to 60 MHz */
				N = 1; /* P = 2 */
				break;
		}
		lpc_private.main_clock = (((freq_sel >> 3) & 0xFF) * 12 * 1000 * 1000);
		/* Setup PLL dividers */
		sys_config->sys_pll_ctrl = (((M - 1) & 0x1F) | (N << 5));
		/* Set sys_pll_clk to internal RC */
		sys_config->sys_pll_clk_sel = LPC_PLL_CLK_SRC_IRC_OSC;
		sys_config->sys_pll_clk_upd_en = 0;  /* SYSPLLCLKUEN must go from LOW to HIGH */
		sys_config->sys_pll_clk_upd_en = 1;
		/* Power-up PLL */
		sys_config->powerdown_run_cfg &= ~(LPC_POWER_DOWN_SYSPLL);
		/* Wait Until PLL Locked */
		while (!(sys_config->sys_pll_status & 0x01));
		/* Use PLL as main clock */
		sys_config->main_clk_sel = LPC_MAIN_CLK_SRC_PLL_OUT;
		/* Switch the main clock source */
		sys_config->main_clk_upd_en = 0;
		sys_config->main_clk_upd_en = 1;
	}

	/* And call all clock updaters */
	propagate_main_clock();
	/* Turn interrupts on once again*/
	lpc_enable_irq();
}

uint32_t get_main_clock(void)
{
	return lpc_private.main_clock;
}

/***************************************************************************** */
/*                     Peripheral Clocks                                       */
/***************************************************************************** */
void Dummy_Clk_Updater(void) {
	do { } while (0);
}

void uart_clk_update(void) __attribute__ ((weak, alias ("Dummy_Clk_Updater")));
void i2c_clock_update(void) __attribute__ ((weak, alias ("Dummy_Clk_Updater")));
void ssp_clk_update(void) __attribute__ ((weak, alias ("Dummy_Clk_Updater")));
void adc_clk_update(void) __attribute__ ((weak, alias ("Dummy_Clk_Updater")));

static void propagate_main_clock(void)
{
	uart_clk_update();
	i2c_clock_update();
	ssp_clk_update();
	adc_clk_update();
}


/***************************************************************************** */
/*                    CLK Out                                                  */
/***************************************************************************** */
/* This is mainly a debug feature, but can be used to provide a clock to an
 * external peripheral */
/* Note that PIO0_12 is the only pin possible for CLK_Out, and is multiplexed
 * with ISP mode selection on reset.
 * The pin must be enabled using a pio table passed to the set_pins() function.
 */

void clkout_on(uint32_t src, uint32_t div)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;

	/* Select clk_out clock source */
	sys_config->clk_out_src_sel = (src & 0x03);
	/* Activate clk_out */
	sys_config->clk_out_div = (div & 0xFF);
	sys_config->clk_out_upd_en = 0;
	sys_config->clk_out_upd_en = 1;
}
void clkout_off(void)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	sys_config->clk_out_div = 0; /* Disable CLKOUT */
	sys_config->clk_out_upd_en = 0;
	sys_config->clk_out_upd_en = 1;
}


/***************************************************************************** */
/*                    Default sleep function                                   */
/***************************************************************************** */
/* Note that if the systick core functions are used these will be overridden */

/* This "msleep" is a simple wait routine which is close to a millisecond sleep
 * whith a CPU Clock of 24MHz, but has no exact relation to time.
 * It is highly dependent to CPU clock speed anyway.
 * Note : This is an active sleep !
 */
static void def_msleep(uint32_t ms)
{
	volatile uint32_t dec = ms * 2667;
	while (dec--);
}

/* Something that's not too far from a microsecond sleep at 24MHz CPU Clock
 * Note : This is an active sleep !
 */
static inline void def_usleep(uint32_t us)
{
	volatile uint32_t dec = us * 2;
	while (dec--);
}

void msleep(uint32_t ms) __attribute__ ((weak, alias ("def_msleep")));
void usleep(uint32_t us) __attribute__ ((weak, alias ("def_usleep")));

