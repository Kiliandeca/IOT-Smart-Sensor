/****************************************************************************
 *  drivers/countertimers.c
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

/* Timers driver for the integrated timers of the LPC122x.
 * The LPC122x Has two 16bits timers and two 32bits timers.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */

#include "core/system.h"
#include "lib/errno.h"
#include "drivers/timers.h"
#include "drivers/countertimers.h"


/* These are local to our file */
struct countertimer_device
{
	struct lpc_timer* regs;
	uint32_t power_bit;
	uint8_t irq;
	uint8_t configured;
	void (*callback)(uint32_t); /* Possible RX callback */
};

static struct countertimer_device countertimers[NUM_COUNTERTIMERS] = {
	{
		.regs = LPC_TMR16B0,
		.power_bit = LPC_SYS_ABH_CLK_CTRL_CT16B0,
		.irq = TIMER0_IRQ,
		.configured = 0,
		.callback = NULL,
	},
	{
		.regs = LPC_TMR16B1,
		.power_bit = LPC_SYS_ABH_CLK_CTRL_CT16B1,
		.irq = TIMER1_IRQ,
		.configured = 0,
		.callback = NULL,
	},
	{
		.regs = LPC_TMR32B0,
		.power_bit = LPC_SYS_ABH_CLK_CTRL_CT32B0,
		.irq = TIMER2_IRQ,
		.configured = 0,
		.callback = NULL,
	},
	{
		.regs = LPC_TMR32B1,
		.power_bit = LPC_SYS_ABH_CLK_CTRL_CT32B1,
		.irq = TIMER3_IRQ,
		.configured = 0,
		.callback = NULL,
	},
};

/* Handlers */
void TIMER_Handler(struct countertimer_device* timer)
{
	uint32_t intr_flags = timer->regs->int_reg; /* Backup the flags */

	/* Clear the interrupt */
	timer->regs->int_reg = intr_flags;
	/* And call the user routine if one has been registered */
	if (timer->callback != NULL) {
		timer->callback(intr_flags);
	}
}
void TIMER_0_Handler(void)
{
	TIMER_Handler(&countertimers[0]);
}
void TIMER_1_Handler(void)
{
	TIMER_Handler(&countertimers[1]);
}
void TIMER_2_Handler(void)
{
	TIMER_Handler(&countertimers[2]);
}
void TIMER_3_Handler(void)
{
	TIMER_Handler(&countertimers[3]);
}



/* Start the timer :
 * Remove the reset flag if present and set timer enable flag.
 * Timer must be turned on and configured (no checks done here).
 */
void countertimer_start(uint8_t timer_num)
{
	/* Remove reset flag and set timer enable flag */
	countertimers[timer_num].regs->timer_ctrl = LPC_TIMER_COUNTER_ENABLE;
}

/* Pause the timer counter, does not reset */
void countertimer_pause(uint8_t timer_num)
{
	/* Remove timer enable flag */
	countertimers[timer_num].regs->timer_ctrl = 0;
}

/* Stops and resets the timer counter */
void countertimer_stop(uint8_t timer_num)
{
	/* Remove timer enable flag and request reset */
	countertimers[timer_num].regs->timer_ctrl = LPC_TIMER_COUNTER_RESET;
	/* Remove reset flag */
	countertimers[timer_num].regs->timer_ctrl = 0;
}

/* Resets the timer and lets it count again imediately */
void countertimer_restart(uint8_t timer_num)
{
	/* Set timer reset flag */
	countertimers[timer_num].regs->timer_ctrl = LPC_TIMER_COUNTER_RESET;
	/* Remove reset flag and start counter */
	countertimers[timer_num].regs->timer_ctrl = LPC_TIMER_COUNTER_ENABLE;
}

int countertimer_get_counter_val(uint8_t timer_num, uint32_t* val)
{
	*val = countertimers[timer_num].regs->timer_counter;
	return 0;
}

int countertimer_get_capture_val(uint8_t timer_num, uint8_t channel, uint32_t* val)
{
	if (channel >= MAX_CHANNELS) {
		return -EINVAL;
	}
	*val = countertimers[timer_num].regs->capture_reg[channel];
	return 0;
}


/* Change the match value of a single timer channel */
int countertimer_set_match(uint8_t timer_num, uint8_t channel, uint32_t val)
{
	if (channel > NUM_COUNTERTIMERS_CHANS)
		return -EINVAL;

	countertimers[timer_num].regs->match_reg[channel] = val;
	return 0;
}

struct common_operations countertimer_ops = {
	.start = countertimer_start,
	.cont = countertimer_start,
	.pause = countertimer_pause,
	.stop = countertimer_stop,
	.restart = countertimer_restart,
	.halt = countertimer_stop,
	.get_counter = countertimer_get_counter_val,
	.get_capture = countertimer_get_capture_val,
	.set_match = countertimer_set_match,
};



/*******************************************************************************/
/* Configuration operations */


/*   Timer Setup as PWM */
/* Returns 0 on success
 * Takes a timer number and a timer PWM config structure as arguments.
 * Refer to timer PWM config structure for details.
 */
int countertimer_pwm_setup(uint8_t timer_num, const struct lpc_timer_pwm_config* conf)
{
	struct countertimer_device* timer = &(countertimers[timer_num]);
	uint8_t max_chan = 0;
	uint8_t active_chans = 0;
	int i = 0;

	/* Make sure we have a PWM cycle length */
	if (conf->period == 0) {
		return -EINVAL;
	}
	switch (timer_num) {
		case LPC_TIMER_16B0:
		case LPC_TIMER_16B1:
			if (conf->nb_channels > NUM_COUNTERTIMERS_16B_PWM_CHANS) {
				return -EINVAL;
			}
			max_chan = NUM_COUNTERTIMERS_16B_PWM_CHANS;
			break;
		case LPC_TIMER_32B0:
		case LPC_TIMER_32B1:
			if (conf->nb_channels > NUM_COUNTERTIMERS_32B_PWM_CHANS) {
				return -EINVAL;
			}
			max_chan = NUM_COUNTERTIMERS_32B_PWM_CHANS;
			break;
	}

	if (conf->period_chan >= MAX_CHANNELS) {
		 return -EINVAL;
	}
	/* Setup selected PWM channels */
	for (i = 0; i < conf->nb_channels; i++) {
		if (conf->outputs[i] >= max_chan) {
			continue;
		}
		timer->regs->match_reg[ conf->outputs[i] ] = conf->match_values[i];
		/* Mark channel as active */
		active_chans |= LPC_PWM_CHANNEL_ENABLE(conf->outputs[i]);
	}

	/* Setup period */
	timer->regs->match_reg[ conf->period_chan ] = conf->period;
	/* Setup selected channel as PWM cycle length control */
	timer->regs->match_ctrl &= ~(LPC_TIMER_MATCH_ERASE(conf->period_chan));
	timer->regs->match_ctrl |= (LPC_TIMER_RESET_ON_MATCH << LPC_TIMER_MATCH_SHIFT(conf->period_chan));
	active_chans |= LPC_PWM_CHANNEL_ENABLE(conf->period_chan);

	/* Setup count mode as timer */
	timer->regs->count_ctrl = LPC_COUNTER_IS_TIMER;

	/* Activate selected PWM channels and period channel */
	timer->regs->pwm_ctrl = active_chans;

	return 0; /* Config OK */
}


/* Timer Setup in timer or counter mode, with optionnal capture and match configuration
 * Takes a timer number and a timer counter config structure as arguments.
 * Returns 0 on success
 */
int countertimer_tc_setup(uint8_t timer_num, const struct lpc_tc_config* conf)
{
	struct countertimer_device* timer = &(countertimers[timer_num]);
	int i = 0;

	if (conf->mode & LPC_TIMER_MODE_SPECIFIC) {
		return -EINVAL;
	}
	/* Erase existing configuration */
	timer->regs->capture_ctrl = 0;
	timer->regs->match_ctrl = 0;
	timer->regs->external_match = 0;

	/* Override the prescaler value if requested */
	if (conf->prescale_val != 0) {
		timer->regs->prescale = conf->prescale_val;
	}
	/* Select between timer (counts on PCLK) and counter mode (counts on CAP events) */
	if (conf->mode & LPC_TIMER_MODE_COUNTER) {
		/* Configure the counter */
		timer->regs->count_ctrl = (conf->count_control & 0x0F);
		timer->regs->count_ctrl |= LPC_COUNTER_INC_INPUT(conf->count_chan);
	} else {
		/* Timer mode */
		timer->regs->count_ctrl = LPC_COUNTER_IS_TIMER;
	}

	/* Configure the reset on capture functionality when selected */
	if (conf->reset_on_cap & LPC_COUNTER_CLEAR_ON_EVENT_EN) {
		timer->regs->count_ctrl = LPC_COUNTER_CLEAR_ON_EVENT_EN;
		timer->regs->count_ctrl |= ((conf->reset_on_cap & 0x07) << LPC_COUNTER_CLEAR_ON_EVENT_SHIFT);
	}

	if (conf->mode & LPC_TIMER_MODE_CAPTURE) {
		for (i = 0; i < MAX_CHANNELS; i++) {
			timer->regs->capture_ctrl |= ((conf->cap_control[i] & 0x07) << LPC_TIMER_CAPTURE_SHIFT(i));
		}
	}

	if (conf->mode & LPC_TIMER_MODE_MATCH) {
		for (i = 0; i < MAX_CHANNELS; i++) {
			timer->regs->match_ctrl |= ((conf->match_control[i] & 0x07) << LPC_TIMER_MATCH_SHIFT(i));
			timer->regs->match_reg[i] = conf->match[i];
			timer->regs->external_match |= ((conf->ext_match_config[i] & 0x03) << LPC_TIMER_EXT_MATCH_SHIFT(i));
		}
	}

	/* Ensure that counter input chan has not been configured to reset or stop the timer or as a
	 * capture channel (See remarks in user manual UM10441 page 268 section 14.7.11) when the ct is
	 * configured in counter mode. */
	if (conf->mode & LPC_TIMER_MODE_COUNTER) {
		timer->regs->match_ctrl &= ~LPC_TIMER_MATCH_ERASE(conf->count_chan);
		timer->regs->capture_ctrl &= ~LPC_TIMER_CAPTURE_ERASE(conf->count_chan);
		timer->regs->external_match &= ~LPC_TIMER_EXT_MATCH_ERASE(conf->count_chan);
	}

	return 0; /* Config OK */
}

struct config_operations countertimer_cfg_ops = {
	.pwm_config = countertimer_pwm_setup,
	.tc_config = countertimer_tc_setup,
};




/*******************************************************************************/
/* Init operations */

/* Power up a timer.
 * clkrate is the desired timer clock rate. It will be used to divide the main clock
 *   to get the timer prescaler value.
 * Set clkrate to 0 to disable the prescaler.
 */
int countertimer_on(uint8_t timer_num, uint32_t clkrate, void (*callback)(uint32_t))
{
	struct countertimer_device* timer = NULL;
	uint32_t prescale; /* The clock divider for the counter */

	timer = &(countertimers[timer_num]);

	NVIC_DisableIRQ( timer->irq );
	/* Power up the timer */
	subsystem_power(timer->power_bit, 1);
	/* Reset counter on next PCLK positive edge, and disable counter */
	timer->regs->timer_ctrl = LPC_TIMER_COUNTER_RESET;

	/* Store the callback, OK even if none given */
	timer->callback = callback;

	/* Set the prescaler value */
	if (clkrate == 0) {
		prescale = 0;
	} else {
		prescale = (get_main_clock() / clkrate) - 1;
	}
	timer->regs->prescale = prescale;

	NVIC_EnableIRQ( timer->irq );
	return 0;
}

/* Removes the main clock from the selected timer block */
int countertimer_off(uint8_t timer_num)
{
	NVIC_DisableIRQ( countertimers[timer_num].irq );
	subsystem_power(countertimers[timer_num].power_bit, 0);
	return 0;
}

struct init_operations countertimer_init_ops = {
	.timer_on = countertimer_on,
	.timer_off = countertimer_off,
};

