/****************************************************************************
 *  core/systick.c
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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
/*               System Tick Timer                                             */
/***************************************************************************** */

/* Driver for the internal systick timer of the LPC122x.
 * Refer to the LPC122x documentation (UM10441.pdf) for more information
 */

#include "core/system.h"
#include "core/systick.h"
#include "lib/errno.h"


/* Static variables */
static volatile uint32_t sleep_count = 0;
static volatile uint32_t tick_ms = 0;
static volatile uint32_t systick_running = 0;
static volatile uint32_t tick_reload = 0;
static uint32_t usleep_us_count = 0;

/* Wraps every 50 days or so with a 1ms tick */
static volatile uint32_t global_wrapping_system_ticks = 0;
/* The systick cycles run at get_main_clock(), and would wrap more often! */
static volatile uint32_t global_wrapping_system_clock_cycles = 0;


struct systick_callback {
	void (*callback) (uint32_t);
	uint16_t period;
	uint16_t countdown;
};
static volatile struct systick_callback cbs[MAX_SYSTICK_CALLBACKS] = {};

/* System Tick Timer Interrupt Handler */
void SysTick_Handler(void)
{
	int i = 0;
	global_wrapping_system_ticks++;
	global_wrapping_system_clock_cycles += tick_reload;
	if (sleep_count != 0) {
		sleep_count--;
	}
	for (i = 0; i < MAX_SYSTICK_CALLBACKS; i++) {
		if (cbs[i].callback != NULL) {
			cbs[i].countdown--;
			if (cbs[i].countdown == 0) {
				cbs[i].countdown = cbs[i].period;
				cbs[i].callback(global_wrapping_system_ticks);
			}
		}
	}
}


/* Register a callback to be called every 'period' system ticks.
 * returns the callback number if registration was OK.
 * returns negative value on error.
 * The callback will get the "global_wrapping_system_ticks" as argument, which wraps every 50 days
 *   or so with a 1ms tick
 */
int add_systick_callback(void (*callback) (uint32_t), uint16_t period)
{
	int i = 0;
	if (period == 0) {
		return -EINVAL;
	}
	for (i = 0; i < MAX_SYSTICK_CALLBACKS; i++) {
		if (cbs[i].callback == NULL) {
			cbs[i].callback = callback;
			cbs[i].period = period;
			cbs[i].countdown = period;
			return i;
		}
	}
	return -EBUSY;
}

int remove_systick_callback(void (*callback) (uint32_t))
{
	int i = 0;
	for (i = 0; i < MAX_SYSTICK_CALLBACKS; i++) {
		if (cbs[i].callback == callback) {
			cbs[i].callback = NULL;
			return 0;
		}
	}
	return -EINVAL;
}

/***************************************************************************** */
/* systick timer control function */

/* Start the system tick timer
 * Starting the systick timer also resets the internal tick counters.
 * If you need a value that goes beyond one start/stop cycle and accross resets,
 *    then it's up to you to keep track of this using systick_get_tick_count() and/or
 *    systick_get_clock_cycles().
 */
void systick_start(void)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	systick->value = 0;
	systick_running = 1;
	global_wrapping_system_ticks = 0;
	global_wrapping_system_clock_cycles = tick_reload;
	systick->control |= LPC_SYSTICK_CTRL_ENABLE;
}
/* Stop the system tick timer */
void systick_stop(void)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	systick->control &= ~(LPC_SYSTICK_CTRL_ENABLE);
	systick_running = 0;
	systick->value = 0;
}
/* Reset the system tick timer, making it count down from the reload value again
 * Reseting the systick timer also resets the internal tick counters.
 * If you need a value that goes beyond one start/stop cycle and accross resets,
 *    then it's up to you to keep track of this using systick_get_tick_count() and/or
 *    systick_get_clock_cycles().
 */
void systick_reset(void)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	systick->value = 0;
	global_wrapping_system_ticks = 0;
	global_wrapping_system_clock_cycles = tick_reload;
}

/* Get system tick timer current value (counts at get_main_clock() !)
 * systick_get_timer_val returns a value between 0 and systick_get_timer_reload_val()
 */
uint32_t systick_get_timer_val(void)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	return systick->value;
}
/* Get system tick timer reload value */
uint32_t systick_get_timer_reload_val(void)
{
	return tick_reload;
}

/* Check if systick is running (return 1) or not (return 0) */
uint32_t is_systick_running(void)
{
	return systick_running;
}

/* Get the system tick period in ms
 * A vaue of 0 means the system tick timer has not been configured.
 * Note : calls to msleep() or usleep() will configure the system tick timer
 *        with a value of 1ms if it was not configured yet.
 */
uint32_t systick_get_tick_ms_period(void)
{
	return tick_ms;
}

/* Get the "timer wrapped" indicator.
 * Used in usleep() function.
 * Note : the first to call this function will get the right information.
 * All subsequent calls will get wrong indication.
 * Thus this function is not exported to user space, user should compare global
 * ticks values or tick counter values. */
static uint32_t systick_counted_to_zero(void)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	return (systick->control & LPC_SYSTICK_CTRL_COUNTFLAG);
}


/* Get the number of system ticks ... since last wrapping of the counter, which
 * is about 50 days with a 1ms system tick. */
uint32_t systick_get_tick_count(void)
{
	return global_wrapping_system_ticks;
}

/* Get the number of clock cycles ... since last wrapping of the counter. */
uint32_t systick_get_clock_cycles(void)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	/* global_wrapping_system_clock_cycles has been initialised to reload value, thus there is
	 * no need to add it here, making the call quicker */
	return global_wrapping_system_clock_cycles - systick->value;
}

/***************************************************************************** */
/* Power up the system tick timer.
 * ms is the interval between system tick timer interrupts. If set to 0, the default
 *     value is used, which should provide a 1ms period.
 */
void systick_timer_on(uint32_t ms)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	uint32_t reload; /* The reload value for the counter */

	/* Set the reload value */
	if (ms != 0) {
		reload = ((get_main_clock() / 1000) * ms) - 1;
	} else {
		reload = (get_main_clock() / 1000) - 1;
		ms = 1;
	}
	/* For the LPC122x the system tick clock is fixed to half the frequency of the system clock */
	reload = reload >> 1; /* Divide by 2 */
	systick->reload_val = (reload & 0xffffff);
	tick_ms = ms;
	tick_reload = systick->reload_val;

	/* Start counting from the reload value, writting anything would do ... */
	systick->value = reload;
	/* Consider we already counted one cycle, making further reading of this count easier */
	global_wrapping_system_clock_cycles = tick_reload;

	/* And enable counter interrupt */
	systick->control = LPC_SYSTICK_CTRL_TICKINT;
	systick_running = 0;

	/* Perform this division now for the usleep function. */
	usleep_us_count = get_main_clock() / (1000 * 1000);
	/* For the LPC122x the system tick clock is fixed to half the frequency of the system clock */
	usleep_us_count = (usleep_us_count >> 1); /* Divide by two */

	/* FIXME : document this */
	NVIC_SetPriority(SYSTICK_IRQ, ((1 << LPC_NVIC_PRIO_BITS) - 1));
}

/* Removes the main clock from the selected timer block */
void systick_timer_off(void)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	systick->control = 0;
	systick->reload_val = 0;
	tick_ms = 0;
	tick_reload = 0;
	systick_running = 0;
}


/***************************************************************************** */
/* Sleeping functions */

/* Set the sleep countdown value
 * A sleep will end when this value reaches 0
 * Note that calls to this function while a sleep() has been initiated will change the
 *   sleep duration ....
 */
static inline void set_sleep(uint32_t ticks)
{
	sleep_count = ticks;
}

/* Actual sleep function, checks that system tick counter is configured to generate
 * an interrupt and to move sleep_count down to 0
 */
#define SYSTICK_CAN_SLEEP   (LPC_SYSTICK_CTRL_TICKINT | LPC_SYSTICK_CTRL_ENABLE)
static uint32_t sleep(void)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	if ((systick->control & SYSTICK_CAN_SLEEP) != SYSTICK_CAN_SLEEP) {
		return -1;
	}
	do { } while (sleep_count != 0);
	return 0;
}

/* This msleep sleeps less than the required amount of time as it forgets about
 * the already elapsed time of the systick timer since last tick. */
void msleep(uint32_t ms)
{
	uint32_t ticks = 0;

	if (tick_ms == 0) {
		systick_timer_on(1);
		ticks = ms;
	} else {
		ticks = ms / tick_ms;
	}
	set_sleep(ticks);
	if (systick_running == 0) {
		systick_start();
	}
	sleep();
}

/* This usleep function tries to sleep at most the required amount of time.
 * The setup is so long that it cannot sleep for less than 10us when running at 48MHz
 */
void usleep(uint32_t us)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	uint32_t start = systick->value; /* Grab the starting (call time) value now */
	uint32_t count = 0;
	uint32_t end = 0;

	end = systick_counted_to_zero(); /* erase loop indicator */
	if (us > 1000) {
		msleep(us / 1000);
		us = us % 1000;
	} else {
		if (systick_running == 0) {
			if (tick_ms == 0) {
				systick_timer_on(1);
			}
			systick_start();
		}
	}
	count = usleep_us_count * us;
	 /* Remember that systick is a decrementing counter */
	if (count > start) {
		end = (systick->reload_val - (count - start));
		do { } while (systick_counted_to_zero() == 0); /* Wait for timer loop */
		do { } while (systick->value > end); /* Wait for remaining part of sleep duration */
	} else {
		end = start - count;
		/* Wait for sleep duration.
		 * If the counter looped, it means we already waited too much */
		do { } while ((systick->value > end) && (systick_counted_to_zero() == 0));
	}
}

void usleep_short(uint32_t us)
{
	struct lpc_system_tick* systick = LPC_SYSTICK;
	uint32_t start = systick->value; /* Grab the starting (call time) value now */
	uint32_t count = usleep_us_count * us;
	uint32_t end = systick_counted_to_zero(); /* Erase loop indicator */

	 /* Remember that systick is a decrementing counter */
	if (count > start) {
		end = (systick->reload_val - (count - start));
		do { } while (systick_counted_to_zero() == 0); /* Wait for timer loop */
		do { } while (systick->value > end); /* Wait for remaining part of sleep duration */
	} else {
		end = start - count;
		/* Wait for sleep duration.
		 * If the counter looped, it means we already waited too much */
		do { } while ((systick->value > end) && (systick_counted_to_zero() == 0));
	}
}

