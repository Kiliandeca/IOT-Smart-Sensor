/****************************************************************************
 *  drivers/rtc.c
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

/***************************************************************************** */
/*                RTC and RTC Interrupts                                    */
/***************************************************************************** */

/* The RTC is an integrated module of the LPC122x.
 * Refer to LPC12xx documentation (UM10441.pdf) for more information
 */

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "drivers/rtc.h"


static uint32_t match_first = 0;
static uint32_t rtc_match_period = 0;
static uint8_t periodic_match = 0;

/***************************************************************************** */
/* Return the number of RTC ticks from system power on.
 * This count is from power being present, even if resets occured.
 * The calls made during the first three seconds after RTC timer start will return 0.
 */
uint32_t rtc_get_count(void)
{
	struct lpc_rtc* rtc = LPC_RTC;
	static uint8_t rtc_start_ok = 0;

	/* Is the count valid ? */
	if (rtc_start_ok == 1) {
		return rtc->data;
	}
	/* Not started for sufficient time to have a reliable value (see UM10441 section 16.6.1) */
	if (is_systick_running() == 0) {
		systick_start();
		return 0;
	} else {
		uint32_t ticks = systick_get_tick_count();
		uint32_t ms_period = systick_get_tick_ms_period();
		if ((ticks * ms_period) < 3000) {
			return 0;
		}
		rtc_start_ok = 1;
		return rtc->data;
	}
}


/***************************************************************************** */
/*   RTC setup   */

/* In case someone wants the RTC to count something different from seconds */
void rtc_clk_src_select(uint8_t source, uint8_t clk_div)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct lpc_pm_unit* pm_unit = LPC_PMU;
	uint32_t tmp = 0;

	/* Turn off RTC controll regs */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_RTC, 0);
	/* Change clock source. Warning : do not write ones to reserved bits. */
	tmp = (pm_unit->system_config & LPC_WAKEUP_PIN_HYST_MASK);
	pm_unit->system_config = (tmp | ((source & 0x0F) << LPC_RTC_CLK_SRC_SHIFT));
	/* Change the RTC Clock divider if source is PCLK */
	if (source == LPC_RTC_CLK_PCLK) {
		sys_config->rtc_clk_div = clk_div;
	} else {
		sys_config->rtc_clk_div = 0;
	}
}

/* Start the RTC. Once the RTC has been started using this call, it cannot be stopped. */
void rtc_on(void)
{
	struct lpc_rtc* rtc = LPC_RTC;
	/* Provide power to RTC control block */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_RTC, 1);
	rtc->control = LPC_RTC_START;
}
/* Disable the RTC control block. Note that once started from software the RTC cannot be stopped. */
void rtc_ctrl_off(void)
{
	/* Remove power from RTC control blockxs */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_RTC, 0);
}
/* This will disable the RTC. This is only possible if the RTC has not been started before. */
void rtc_disable(void)
{
	struct lpc_rtc* rtc = LPC_RTC;
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_RTC, 1);
	rtc->control = LPC_RTC_DISABLE;
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_RTC, 0);
}

/* Change the RTC value */
void rtc_set_date(uint32_t date)
{
	struct lpc_rtc* rtc = LPC_RTC;
	rtc->load = date;
}


/***************************************************************************** */
/* Set the match value for the RTC
 * If periodic is 1 then the rtc match register will be updated after the interrupt
 *    has been handled by adding the offset.
 * If date is set, the match register will be set to date. (regardless of the periodic
 *    argument, which allows to have the first interrupt at a given date, and the following
 *    ones at a period set by the offset argument.
 * If date is 0, then the match register is set to current date + offset.
 * return -1 if RTC is not started and synced
 */
int rtc_set_match(uint32_t date, uint32_t offset, uint8_t periodic)
{
	struct lpc_rtc* rtc = LPC_RTC;

	if (rtc_get_count() == 0) {
		return -1;
	}

	periodic_match = periodic;
	rtc_match_period = offset;
	if (date != 0) {
		rtc->match = date;
	} else {
		rtc->match = rtc_get_count() + offset;
	}
	return 0;
}

/* RTC Interrupts Callbacks */
static void (*rtc_callback) (uint32_t);

void setup_callback(uint32_t ticks)
{
	uint32_t date = rtc_get_count();
	if (date != 0) {
		if (date >= match_first) {
			match_first = 0; /* First match missed, forget about it */
		}
		rtc_set_match(match_first, rtc_match_period, periodic_match);
		remove_systick_callback(&setup_callback);
	}
}

/* Register a callback for the RTC
 * 'when' should be in the future (according to RTC counter) and 'period' can be used to get
 *   periodic interrupts. 'period' is a number of RTC counter increments.
 * Return a positive integer if registration is OK and callback has a chance of being called.
 * Return a negative integer if the match configuration was not possible.
 * Note :
 *  The callback becomes active only after the RTC starts returning valid data. set_rtc_callback()
 *  takes care of waiting for this event by installing a systick callback which tries to setup the
 *  RTC callback every RTC_CB_INST_RETRY_MS milli-seconds. This systick callback will be removed
 *  once the RTC callback is installed.
 */
int set_rtc_callback(void (*callback) (uint32_t), uint32_t when, uint32_t period)
{
	struct lpc_rtc* rtc = LPC_RTC;
	int ret = 0;

	/* Register the callback */
	rtc_callback = callback;
	if (rtc_get_count() != 0) {
		rtc_set_match(when, period, ((period == 0) ? 0 : 1));
	} else {
		/* Add callback for "config later" */
		uint32_t retry_period = (RTC_CB_INST_RETRY_MS / systick_get_tick_ms_period());
		ret = add_systick_callback(&setup_callback, retry_period);
		match_first = when;
		periodic_match = ((period == 0) ? 0 : 1);
		rtc_match_period = period;
	}
	rtc->intr_control = LPC_RTC_INT_ENABLE;
	NVIC_EnableIRQ(RTC_IRQ);
	return ret;
}
void remove_rtc_callback()
{
	struct lpc_rtc* rtc = LPC_RTC;

	/* Remove the handler */
	rtc_callback = NULL;
	rtc->intr_control = LPC_RTC_INT_DISABLE;
	/* And disable the interrupt */
	NVIC_DisableIRQ(RTC_IRQ);
}

/* Interrupt Handler */
void RTC_Handler(void)
{
	struct lpc_rtc* rtc = LPC_RTC;
	/* Call interrupt handler */
	if (rtc_callback != NULL) {
		rtc_callback(rtc->data);
	}
	rtc->intr_clear = LPC_RTC_CLEAR_INTR;
	if (periodic_match == 1) {
		rtc->match = rtc->data + rtc_match_period;
	}
}



