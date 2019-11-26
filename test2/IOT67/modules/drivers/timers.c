/****************************************************************************
 *  drivers/timers.c
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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
/*                Timers                                                       */
/***************************************************************************** */

/* Driver for the different kinds of timers available in the LPC122x
 *
 * This inludes :
 * - 32 bits Timers driver
 *   The LPC122x has two 32 bits Timer.
 * - 16 bits Timers driver
 *   The LPC122x has two 16 bits Timer.
 *
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 *
 * All common functions are available using a common interface. Only specific
 * functions have a specific interface, described in separate headers files.
 */


#include "lib/stdint.h"
#include "lib/errno.h"
#include "drivers/timers.h"


/* These are local to our file */
struct timer_device {
	uint8_t num;
	struct common_operations* ops;
	struct config_operations* cfg_ops;
	struct init_operations* init_ops;
};


/* 32 bits and 16 bits Counter Timers */
extern struct common_operations countertimer_ops;
extern struct config_operations countertimer_cfg_ops;
extern struct init_operations countertimer_init_ops;


struct timer_device timers[NUM_TIMERS] = {
	{
		.num = LPC_TIMER_16B0,
		.ops = &countertimer_ops,
		.cfg_ops = &countertimer_cfg_ops,
		.init_ops = &countertimer_init_ops,
	},
	{
		.num = LPC_TIMER_16B1,
		.ops = &countertimer_ops,
		.cfg_ops = &countertimer_cfg_ops,
		.init_ops = &countertimer_init_ops,
	},
	{
		.num = LPC_TIMER_32B0,
		.ops = &countertimer_ops,
		.cfg_ops = &countertimer_cfg_ops,
		.init_ops = &countertimer_init_ops,
	},
	{
		.num = LPC_TIMER_32B1,
		.ops = &countertimer_ops,
		.cfg_ops = &countertimer_cfg_ops,
		.init_ops = &countertimer_init_ops,
	},
};


/*******************************************************************************/
/* Wrappers to common functions */

#define GENERIC_TIMER_OPS(name) \
	void timer_ ## name(uint8_t timer_num) \
	{ \
		if (timer_num >= NUM_TIMERS) \
			return; \
		if (timers[timer_num].ops && timers[timer_num].ops->name) { \
			timers[timer_num].ops->name(timer_num); \
		} \
	}

/* Start a timer */
GENERIC_TIMER_OPS(start);
/* Pause timer operation */
GENERIC_TIMER_OPS(pause);
/* Continue timer operation when the timer got paused */
GENERIC_TIMER_OPS(cont);
/* Reset the timer and let it count again imediately */
GENERIC_TIMER_OPS(restart);
/* Stop timer counting and reset timer counter to reload value / initial state */
GENERIC_TIMER_OPS(stop);
/* Same as stop */
GENERIC_TIMER_OPS(halt);


/* Get the current counter value
 * Return 0 if the value is valid.
 */
int timer_get_counter_val(uint8_t timer_num, uint32_t* val)
{
	if (timer_num >= NUM_TIMERS)
		return -EINVAL;
	if (timers[timer_num].ops && timers[timer_num].ops->get_counter) {
		return timers[timer_num].ops->get_counter(timer_num, val);
	}
	return -ENODEV;
}


/* Get the value of the timer when the capture event last triggered
 * Return 0 if the value is valid.
 */
int timer_get_capture_val(uint8_t timer_num, uint8_t channel, uint32_t* val)
{
	if (timer_num >= NUM_TIMERS)
		return -EINVAL;
	if (timers[timer_num].ops && timers[timer_num].ops->get_capture) {
		return timers[timer_num].ops->get_capture(timer_num, channel, val);
	}
	return -ENODEV;
}


/* Change the match value of a single timer channel */
int timer_set_match(uint8_t timer_num, uint8_t channel, uint32_t val)
{
	if (timer_num >= NUM_TIMERS)
		return -EINVAL;
	if (timers[timer_num].ops && timers[timer_num].ops->set_match) {
		return timers[timer_num].ops->set_match(timer_num, channel, val);
	}
	return -ENODEV;
}


/*******************************************************************************/
/* Configuration */

/* Configure the timer as PWM. Call to timer-specific function */
int timer_pwm_config(uint8_t timer_num, const struct lpc_timer_pwm_config* conf)
{
	if (timer_num >= NUM_TIMERS)
		return -EINVAL;
	if (timers[timer_num].cfg_ops && timers[timer_num].cfg_ops->pwm_config) {
		return timers[timer_num].cfg_ops->pwm_config(timer_num, conf);
	}
	return -ENODEV;
}


/* Timer Setup in timer or counter mode, with optionnal capture and match configuration
 * Takes a timer number and a timer counter config structure as arguments.
 * Returns 0 on success
 */
int timer_counter_config(uint8_t timer_num, const struct lpc_tc_config* conf)
{
	if (timer_num >= NUM_TIMERS)
		return -EINVAL;
	if (timers[timer_num].cfg_ops && timers[timer_num].cfg_ops->tc_config) {
		return timers[timer_num].cfg_ops->tc_config(timer_num, conf);
	}
	return -ENODEV;
}

/*******************************************************************************/
/* Init operations */

/* Power up a timer.
 * clkrate is the desired timer clock rate. It will be used to divide the main clock
 *   to get the timer prescaler value.
 *   Set clkrate to 0 to disable the prescaler.
 * callback is the interrupt callback for this timer.
 */
int timer_on(uint8_t timer_num, uint32_t clkrate, void (*callback)(uint32_t))
{
	if (timer_num >= NUM_TIMERS)
		return -EINVAL;
	if (timers[timer_num].init_ops && timers[timer_num].init_ops->timer_on) {
		timers[timer_num].init_ops->timer_on(timer_num, clkrate, callback);
	}
	return -ENODEV;
}

/* Removes the main clock from the selected timer block */
int timer_off(uint8_t timer_num)
{
	if (timer_num >= NUM_TIMERS)
		return -EINVAL;
	if (timers[timer_num].init_ops && timers[timer_num].init_ops->timer_off) {
		return timers[timer_num].init_ops->timer_off(timer_num);
	}
	return -ENODEV;
}

