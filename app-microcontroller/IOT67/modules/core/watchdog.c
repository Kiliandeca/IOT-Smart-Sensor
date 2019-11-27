/****************************************************************************
 *   core/watchdog.c
 *
 * Watchdog support
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
 * This file implements support of the Windowed Watchdog (WWDT)
 */

#include "core/system.h"
#include "core/watchdog.h"



/***************************************************************************** */
void watchdog_feed(void)
{
	struct lpc_watchdog* wdt = LPC_WDT;
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_Watchdog, 1);
	lpc_disable_irq();
	wdt->feed_seqence = 0xAA;
	wdt->feed_seqence = 0x55;
	lpc_enable_irq();
}

static void (*wdt_callback)(void) = NULL;

void WDT_Handler(void)
{
	struct lpc_watchdog* wdt = LPC_WDT;
	wdt->mode |= LPC_WDT_INTR_FLAG;
	/* Call user callback if the user registered one */
	if (wdt_callback != NULL) {
		wdt_callback();
	}
}

/* Lock the watchdog clock source. Once the clock is locked, the configuration is
 * permanent, there's no way to change it. Make all configuration steps before locking
 * the watchdog Clock source.
*/
void watchdog_lock_clk_src(void)
{
	struct lpc_watchdog* wdt = LPC_WDT;
	wdt->clk_src_sel |= LPC_WDT_CLK_SRC_LOCK;
}

/* Lock the watchdog clock source power.
 * Once locked, writes to the current watchdog clock power bits in powerdown_*_cfg will
 *   have no effect.
 * It is still possible to switch the watchdog clock source and turn of the clock if the
 *   watchdog clock source has not been locked yet.
 */
void watchdog_lock_clk_src_power(void)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct lpc_watchdog* wdt = LPC_WDT;

	if (wdt->clk_src_sel & LPC_WDT_CLK_WDOSC) {
		sys_config->powerdown_sleep_cfg &= ~(LPC_POWER_DOWN_WDT_OSC);
		sys_config->powerdown_wake_cfg &= ~(LPC_POWER_DOWN_WDT_OSC);
	} else {
		sys_config->powerdown_wake_cfg &= ~(LPC_POWER_DOWN_IRC);
		sys_config->powerdown_wake_cfg &= ~(LPC_POWER_DOWN_IRC_OUT);
	}
	wdt->mode |= LPC_WDT_CLK_POWER_LOCK;
}

/* Lock the watchdog timer value */
void watchdog_lock_timer_val(void)
{
	struct lpc_watchdog* wdt = LPC_WDT;
	wdt->mode |= LPC_WDT_TIMER_VAL_PROTECT;
}

/* Change the watchdog timer value, if not protected */
void watchdog_set_timer_val(uint32_t nb_clk)
{
	struct lpc_watchdog* wdt = LPC_WDT;

	if (!(wdt->mode & LPC_WDT_TIMER_VAL_PROTECT)) {
		wdt->timer_const = ((nb_clk >> 2) & LPC_WDT_TIMER_MAX);
	}
}

/* Lock the Watchdog enable bit.
 * It is still possible to disable the watchdog by setting it's clock to an unpowered
 *   source if you did not lock the watchdog clock source and clock source power.
 */
void watchdog_lock_enable(void)
{
	struct lpc_watchdog* wdt = LPC_WDT;
	wdt->mode |= LPC_WDT_EN_LOCK;
}
/* Lock the watchdog and all related features.
 * Calls all the other watchdog_lock_* functions (clk_src, clk_src_power, timer_val, enable).
 */
void watchdog_lock_full(void)
{
	watchdog_lock_enable();
	watchdog_lock_timer_val();
	watchdog_lock_clk_src_power();
	watchdog_lock_clk_src();
}

/* Disable deep power down mode entry
 * Calls to wfi() will allow entry in sleep and deep-sleep modes, but not deep-power-down mode.
 */
void watchdog_disable_power_down(void)
{
	struct lpc_watchdog* wdt = LPC_WDT;
	wdt->mode |= LPC_WDT_POWER_DOWN_DISABLE;
}

/*
 * Configure the watchdog.
 * clk_sel is either 0 (IRC) or 1 (WDTCLK). The corresponding clock source will be powered on.
 * Note : only WDTCLK is running in deep power down mode
 * Note : protecting the clock source power will prevent turning off the IRC for power saving
 *   if it is selected as main clock source.
 */
void watchdog_config(const struct wdt_config* wd_conf)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct lpc_watchdog* wdt = LPC_WDT;

	NVIC_DisableIRQ(WDT_IRQ);
	/* Power wadchdog block before changing it's configuration */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_Watchdog, 1);
	/* If intr_mode_only is set, a watchdog timeout will trigger an interrupt instead of a reset */
	if (wd_conf->intr_mode_only == 1) {
		wdt->mode = LPC_WDT_EN;
	} else {
		wdt->mode = LPC_WDT_EN | LPC_WDT_RESET_ON_TIMEOUT;
	}
	/* Register the callback for the interrupt */
	wdt_callback = wd_conf->callback;
	/* Configure watchdog timeout for normal operation */
	wdt->timer_const = ((wd_conf->nb_clk >> 2) & LPC_WDT_TIMER_MAX);
	/* Watchdog clock select */
	if (wd_conf->clk_sel == LPC_WDT_CLK_IRC) {
		sys_config->powerdown_run_cfg &= ~(LPC_POWER_DOWN_IRC);
		sys_config->powerdown_run_cfg &= ~(LPC_POWER_DOWN_IRC_OUT);
		wdt->clk_src_sel = LPC_WDT_CLK_IRC;
	} else {
		sys_config->powerdown_run_cfg &= ~(LPC_POWER_DOWN_WDT_OSC);
		wdt->clk_src_sel = LPC_WDT_CLK_WDOSC;
	}
	/* Use the windows functionnality ? */
	if (wd_conf->wdt_window > 0x100) {
		wdt->window_compare = (wd_conf->wdt_window & LPC_WDT_TIMER_MAX);
	}
	/* Warning interrupt ? */
	if (wd_conf->wdt_warn != 0) {
		if (wd_conf->wdt_warn > LPC_WDT_WARNINT_MAXVAL) {
			wdt->warning_int_compare = LPC_WDT_WARNINT_MAXVAL;
		} else {
			wdt->warning_int_compare = wd_conf->wdt_warn;
		}
	}
	/* Protect any of the watchdog functions now ? */
	if (wd_conf->locks != 0) {
		uint32_t mode = wdt->mode;
		if (wd_conf->locks & WDT_CLK_POWER_LOCK) {
			mode |= LPC_WDT_CLK_POWER_LOCK;
			if (wd_conf->clk_sel == LPC_WDT_CLK_WDOSC) {
				sys_config->powerdown_sleep_cfg &= ~(LPC_POWER_DOWN_WDT_OSC);
				sys_config->powerdown_wake_cfg &= ~(LPC_POWER_DOWN_WDT_OSC);
			} else {
				sys_config->powerdown_wake_cfg &= ~(LPC_POWER_DOWN_IRC);
				sys_config->powerdown_wake_cfg &= ~(LPC_POWER_DOWN_IRC_OUT);
			}
		}
		if (wd_conf->locks & WDT_CLK_SRC_LOCK) {
			wdt->clk_src_sel |= LPC_WDT_CLK_SRC_LOCK;
		}
		if (wd_conf->locks & WDT_EN_LOCK) {
			mode |= LPC_WDT_EN_LOCK;
		}
		if (wd_conf->locks & WDT_TIMER_VAL_LOCK) {
			mode |= LPC_WDT_TIMER_VAL_PROTECT;
		}
		if (wd_conf->locks & WDT_POWER_DOWN_LOCK) {
			mode |= LPC_WDT_POWER_DOWN_DISABLE;
		}
		wdt->mode = mode;
	}
	/* Feed sequence to validate the configuration */
	watchdog_feed();
	NVIC_EnableIRQ(WDT_IRQ);
}


/*
 * Stop the watchdog
 * This function can be used during system operation to stop the watchdog if it has not
 *   been locked or protected against clock source modification, for example when entering
 *   sleep or deep sleep.
 * It will also try to power-down the oscilators if not used for main clock.
 * Return 0 if a solution has been found to stop the watchdog, or -1 if watchdog is still
 *   running after this call.
 * TODO : Check this function, and implement the missing cases
 */
int stop_watchdog(void)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct lpc_watchdog* wdt = LPC_WDT;
	int ret = -1;

	NVIC_DisableIRQ(WDT_IRQ);
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_Watchdog, 1);
	/* Clear enable bit ? */
	if (!(wdt->mode & LPC_WDT_EN_LOCK)) {
		wdt->mode &= ~(LPC_WDT_EN);
		watchdog_feed();
		ret = 0;
	} else if (!(wdt->clk_src_sel & LPC_WDT_CLK_SRC_LOCK)) {
		/* If Watchdog enable bit cannot be cleared, try to set watchdog clock to
		 *   an unpowered clock source */
		/* If current clock is WDCLK and power to curent clock is protected, temprarily
		 *   move clk source to IRC */
		if ((wdt->clk_src_sel == LPC_WDT_CLK_WDOSC) && (wdt->mode & LPC_WDT_CLK_POWER_LOCK)) {
			wdt->clk_src_sel = LPC_WDT_CLK_IRC;
		}
		sys_config->powerdown_run_cfg |= LPC_POWER_DOWN_WDT_OSC;
		/* Power wadchdog block before changing it's configuration */
		wdt->clk_src_sel = LPC_WDT_CLK_WDOSC;
		ret = 0;
	}
	if (ret != 0) {
		subsystem_power(LPC_SYS_ABH_CLK_CTRL_Watchdog, 0);
		return -1;
	}
	if (wdt->mode & LPC_WDT_CLK_POWER_LOCK) {
		subsystem_power(LPC_SYS_ABH_CLK_CTRL_Watchdog, 0);
		return 0;
	}
	/* If main clock and clkout not running from IRC (possibly through PLL), turn off IRC */
	if ((sys_config->main_clk_sel != LPC_MAIN_CLK_SRC_IRC_OSC) &&
		((sys_config->main_clk_sel & 0x01) && (sys_config->sys_pll_clk_sel != LPC_PLL_CLK_SRC_IRC_OSC)) &&
		(sys_config->clk_out_src_sel != LPC_CLKOUT_SRC_IRC_OSC)) {
		sys_config->powerdown_run_cfg |= LPC_POWER_DOWN_IRC;
		sys_config->powerdown_run_cfg |= LPC_POWER_DOWN_IRC_OUT;
	}
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_Watchdog, 0);
	return 0;
}



/*
 * Disable the watchdog
 * This function can be used upon system startup to disable watchdog operation
 */
void startup_watchdog_disable(void)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct lpc_watchdog* wdt = LPC_WDT;

	/* Power wadchdog block before changing it's configuration */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_Watchdog, 1);
	/* Stop watchdog */
	wdt->mode = 0;
	watchdog_feed();
	/* And power it down */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_Watchdog, 0);
	sys_config->powerdown_run_cfg |= LPC_POWER_DOWN_WDT_OSC;
	NVIC_DisableIRQ(WDT_IRQ);
}


