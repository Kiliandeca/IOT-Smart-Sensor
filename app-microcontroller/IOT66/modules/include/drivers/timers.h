/****************************************************************************
 *  drivers/timers.h
 *
 * Copyright 2012-2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef DRIVERS_TIMERS_H
#define DRIVERS_TIMERS_H

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
#include "drivers/countertimers.h"

/***************************************************************************** */
/* All timers have 4 channels. 32 bits timers have all 4 channels available
 *   on capture / match pins while 16 bits timers have only two (channels 0 and 1).
 */
#define NUM_TIMERS 4
#define MAX_CHANNELS 4

/* Timer numbers to be used for functions from this driver. */
enum lpc_timers {
	LPC_TIMER_16B0 = 0,  /* 16 bits timer 0 */
	LPC_TIMER_16B1,  /* 16 bits timer 1 */
	LPC_TIMER_32B0,  /* 32 bits timer 0 */
	LPC_TIMER_32B1,  /* 32 bits timer 1 */
};

enum lpc_timer_channels {
	CHAN0 = 0,
	CHAN1,
	CHAN2,
	CHAN3,
};

/* Available timer modes *
 * Some mode may be combined, and some are exlusive :
 * Values here are important :
 *   PWM and PWD and any further mode handled by a specific configure function will have
 *   bit 2 set
 */
#define LPC_TIMER_MODE_TIMER     (0x00 << 0)
#define LPC_TIMER_MODE_COUNTER   (0x01 << 0)
#define LPC_TIMER_MODE_SPECIFIC  (0x02 << 0)
#define LPC_TIMER_MODE_CAPTURE   (0x01 << 6)
#define LPC_TIMER_MODE_MATCH     (0x01 << 7)
/* Specific modes */
#define LPC_TIMER_MODE_PWM       (LPC_TIMER_MODE_SPECIFIC | (0x01 << 2))  /* Pulse Width Modulation */
#define LPC_TIMER_MODE_PWD       (LPC_TIMER_MODE_SPECIFIC | (0x02 << 2))  /* Pulse Width Demodulation */


/*******************************************************************************/
/* Configurations */

/* PWM
 * This structure is used for the timer_pwm_config() function.
 * nb_channels is the number of PWM channels used
 * period_chan is the channel number used to generate the period.
 * period is the PWM cycle period, in timer clock cycles. It is the value to wich the timer
 *   will count.
 * outputs[] is the list of outputs / channels corresponding to each 'match_values'
 * match_values[] control the PWM channels duty-cycle. They are the timer values at wich
 *   the corresponding outputs will toggle. They must be lower than or equal to the period
 *   value.
 * outputs_initial_state defines whether the outputs start low or high at the beginning of
 *   the PWM cycle. Support for this depends on the target.
 *   The LPC122x enforces a low state initial output.
 */
struct lpc_timer_pwm_config {
	uint8_t nb_channels;
	uint8_t period_chan;
	uint32_t period;
	uint8_t outputs[MAX_CHANNELS];
	uint32_t match_values[MAX_CHANNELS];
	uint32_t outputs_initial_state;
};

/* This structure is used for the timer_counter_config() function.
 * mode is a mask of LPC_TIMER_MODE_* values. Note that some can be combined, and some are
 *   exclusive : only one of these : LPC_TIMER_MODE_TIMER or LPC_TIMER_MODE_COUNTER and
 *   possibly one or both of these : LPC_TIMER_MODE_CAPTURE and LPC_TIMER_MODE_MATCH
 * prescale_val: if not nul, override the prescaler value computed during timer_on() with this
 *   value. It is not clear in the documentation whether the prescaler is used in counter mode
 *   or not.
 * count_control selects the capture channel used to increment the counter when in counter mode.
 *   count_control is one of LPC_COUNTER_INC_ON_RISING or LPC_COUNTER_INC_ON_FALLING or
 *   LPC_COUNTER_INC_ON_BOTH.
 * count_chann selects the channel used to increment the counter (value between 0 and MAX_CHANS).
 * reset_on_cap is activated by adding LPC_COUNTER_CLEAR_ON_EVENT_EN to a capture reset event (one
 *   of LPC_COUNTER_CLEAR_ON_*** whith *** Going from CHAN0_RISE, CHAN0_FALL, ... to CHAN3_FALL)
 * match_control[] is a combination of LPC_TIMER_INTERRUPT_ON_MATCH, LPC_TIMER_RESET_ON_MATCH and
 *   LPC_TIMER_STOP_ON_MATCH for each match channel.
 *   This controls the internal behaviour on each match event.
 * match[] holds the match values.
 * ext_match_config[] is one of LPC_TIMER_NOTHING_ON_MATCH, LPC_TIMER_CLEAR_ON_MATCH,
 *   LPC_TIMER_SET_ON_MATCH or LPC_TIMER_TOGGLE_ON_MATCH for each match channel.
 *   This controls the behavior of the output associated to the corresponding match channel. The
 *   corresponding pin must be configured as match output by the user.
 * cap_control[] is a combination of LPC_TIMER_CAP_ON_RISING_EDGE, LPC_TIMER_CAP_ON_FALLING_EDGE
 *   and LPC_TIMER_INTERRUPT_ON_CAPTURE.
 *   This controls when the TC current value is loaded to the corresponding match value register,
 *   and whether this triggers an interrupt or not.
 */
struct lpc_tc_config {
	uint16_t mode;
	uint32_t prescale_val;
	uint8_t count_control;
	uint8_t count_chan;
	uint8_t reset_on_cap;
	uint8_t match_control[MAX_CHANNELS];
	uint32_t match[MAX_CHANNELS];
	uint8_t ext_match_config[MAX_CHANNELS];
	uint8_t cap_control[MAX_CHANNELS];
};



/*******************************************************************************/
/* Operation structures, not for use by user programms */
struct common_operations {
	/* Running control */
	void (*start)(uint8_t);
	void (*stop)(uint8_t);
	void (*pause)(uint8_t);
	void (*cont)(uint8_t);
	void (*restart)(uint8_t);
	void (*halt)(uint8_t);
	/* Counter */
	int (*get_counter)(uint8_t, uint32_t*);
	int (*get_capture)(uint8_t, uint8_t, uint32_t*);
	int (*set_match)(uint8_t, uint8_t, uint32_t);
};
struct config_operations {
	int (*pwm_config)(uint8_t, const struct lpc_timer_pwm_config*);
	int (*tc_config)(uint8_t, const struct lpc_tc_config*);
};
struct init_operations {
	int (*timer_on)(uint8_t, uint32_t, void (*)(uint32_t));
	int (*timer_off)(uint8_t);
};

/*******************************************************************************/
/* Common operations */

/* Start the timer :
 * Remove the reset flag if present and set timer enable flag.
 * Timer must be turned on and configured (no checks done here).
 */
void timer_start(uint8_t timer_num);
void timer_continue(uint8_t timer_num);

/* Pause the timer counter, does not reset */
void timer_pause(uint8_t timer_num);

/* Stop and reset the timer counter */
void timer_stop(uint8_t timer_num);
void timer_halt(uint8_t timer_num);

/* Resets the timer and lets it count again imediately */
void timer_restart(uint8_t timer_num);


/* Get the current counter value
 * Return 0 if the value is valid.
 */
int timer_get_counter_val(uint8_t timer_num, uint32_t* val);


/* Get the value of the timer when the capture event last triggered
  * Return 0 if the value is valid.
 */
int timer_get_capture_val(uint8_t timer_num, uint8_t channel, uint32_t* val);

/* Change the match value of a single timer channel */
int timer_set_match(uint8_t timer_num, uint8_t channel, uint32_t val);


/*******************************************************************************/
/* Configuration */

/* PWM configuration in order to use the timer as a PWM controller.
 * For the pwm_conf structure, refer to it's definition above..
 * The Timer must be "on" (call timer_on() for this timer before the call to
 *   timer_pwm_config(), with the disired clock rate, which will define the
 *   length of a timer clock cycle, which is the base for the timer period
 *   definition).
 * The timer will not be started. User code must call timer_start() in order
 *   to start the PWM.
 */
int timer_pwm_config(uint8_t timer_num, const struct lpc_timer_pwm_config* pwm_conf);


/* Timer Setup in timer or counter mode, with optionnal capture and match configuration
 * Takes a timer number and a timer counter config structure as arguments.
 * Returns 0 on success
 */
int timer_counter_config(uint8_t timer_num, const struct lpc_tc_config* conf);


/*******************************************************************************/
/* Init */

/* Power up a timer.
 * clkrate is the desired timer clock rate. It will be used to divide the main clock
 *   to get the timer prescaler value.
 *   Set clkrate to 0 to disable the prescaler.
 * callback is used for all the possible timer interrupts (activated using the
 *   config field in timer_config struct upon timer setup)
 *   The interrupt flags are passed to the interrupt routine as argument.
 */
int timer_on(uint8_t timer_num, uint32_t clkrate, void (*callback)(uint32_t));

/* Removes the main clock from the selected timer block */
int timer_off(uint8_t timer_num);


#endif /* DRIVERS_TIMERS_H */

