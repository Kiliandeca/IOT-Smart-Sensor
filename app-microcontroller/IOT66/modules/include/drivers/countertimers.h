/****************************************************************************
 *  drivers/countertimers.h
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

#ifndef DRIVERS_COUNTER_TIMERS_H
#define DRIVERS_COUNTER_TIMERS_H


#include "lib/stdint.h"
#include "core/lpc_regs.h"

/***************************************************************************** */
/*                  Counter Timer                                              */
/***************************************************************************** */
/* Timers driver for the integrated timers of the LPC122x.
 * The LPC122x has two 16bits timers and two 32bits timers.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */
#define NUM_COUNTERTIMERS     4
#define NUM_COUNTERTIMERS_CHANS  4
#define NUM_COUNTERTIMERS_16B_PWM_CHANS 2
#define NUM_COUNTERTIMERS_32B_PWM_CHANS 4

/* Timer (TMR) */
struct lpc_timer
{
	volatile uint32_t int_reg;        /* 0x000 : Interrupt Register (R/W) */
	volatile uint32_t timer_ctrl;     /* 0x004 : Timer Control Register (R/W) */
	volatile uint32_t timer_counter;  /* 0x008 : Timer Counter Register (R/W) */
	volatile uint32_t prescale;       /* 0x00C : Prescale Register (R/W) */
	volatile uint32_t prescale_counter;  /* 0x010 : Prescale Counter Register (R/W) */
	volatile uint32_t match_ctrl;     /* 0x014 : Match Control Register (R/W) */
	volatile uint32_t match_reg[4];    /* 0x018 : Match Register 0 to 3 (R/W) */
	volatile uint32_t capture_ctrl;   /* 0x028 : Capture Control Register (R/W) */
	volatile const uint32_t capture_reg[4]; /* 0x02C : Capture Register 0 to 3 (R/ ) */
	volatile uint32_t external_match; /* 0x03C : External Match Register (R/W) */
	uint32_t reserved_2[12];
	volatile uint32_t count_ctrl;     /* 0x070 : Count Control Register (R/W) */
	volatile uint32_t pwm_ctrl;       /* 0x074 : PWM Control Register (R/W) */
};
#define LPC_TMR16B0     ((struct lpc_timer *) LPC_TIMER0_BASE)
#define LPC_TMR16B1     ((struct lpc_timer *) LPC_TIMER1_BASE)
#define LPC_TMR32B0     ((struct lpc_timer *) LPC_TIMER2_BASE)
#define LPC_TMR32B1     ((struct lpc_timer *) LPC_TIMER3_BASE)
#define LPC_TIMER_REGS(x)  ((struct lpc_timer *) (LPC_TIMER0_BASE + ((x) * 0x4000)))


#define LPC_TIMER_COUNTER_ENABLE (1 << 0) /* CEN */
#define LPC_TIMER_COUNTER_RESET  (1 << 1) /* CRST */


/* Match internal configuration */
#define LPC_TIMER_INTERRUPT_ON_MATCH   0x01
#define LPC_TIMER_RESET_ON_MATCH       0x02
#define LPC_TIMER_STOP_ON_MATCH        0x04
#define LPC_TIMER_INT_RESET_AND_STOP_ON_MATCH  \
			(LPC_TIMER_INTERRUPT_ON_MATCH | LPC_TIMER_RESET_ON_MATCH | LPC_TIMER_STOP_ON_MATCH)
#define LPC_TIMER_MATCH_SHIFT(x)       (((x) & 0x03) * 3)
#define LPC_TIMER_MATCH_ERASE(x)       (0x07 << LPC_TIMER_MATCH_SHIFT(x))

/* Capture internal configuration */
#define LPC_TIMER_CAP_ON_RISING_EDGE   0x01
#define LPC_TIMER_CAP_ON_FALLING_EDGE  0x02
#define LPC_TIMER_INTERRUPT_ON_CAPTURE 0x04
#define LPC_TIMER_CAPTURE_SHIFT(x)     (((x) & 0x03) * 3)
#define LPC_TIMER_CAPTURE_ERASE(x)     (0x07 << LPC_TIMER_CAPTURE_SHIFT(x))

/* Match external configuration */
#define LPC_TIMER_NOTHING_ON_MATCH     0x00
#define LPC_TIMER_CLEAR_ON_MATCH       0x01
#define LPC_TIMER_SET_ON_MATCH         0x02
#define LPC_TIMER_TOGGLE_ON_MATCH      0x03
#define LPC_TIMER_EXT_MATCH_SHIFT(x)   (((x) * 2)  + 4)
#define LPC_TIMER_EXT_MATCH_ERASE(x)   (0x03 << LPC_TIMER_EXT_MATCH_SHIFT(x))

/* Counter */
#define LPC_COUNTER_IS_TIMER           0x00
#define LPC_COUNTER_INC_ON_RISING      0x01
#define LPC_COUNTER_INC_ON_FALLING     0x02
#define LPC_COUNTER_INC_ON_BOTH        0x03
#define LPC_COUNTER_INC_INPUT_SHIFT    2
#define LPC_COUNTER_INC_INPUT(x)       (((x) & 0x03) << LPC_COUNTER_INC_INPUT_SHIFT)
#define LPC_COUNTER_CLEAR_ON_EVENT_EN  (0x01 << 4)
#define LPC_COUNTER_CLEAR_ON_EVENT_SHIFT  5
#define LPC_COUNTER_CLEAR_ON_CHAN0_RISE   0x00
#define LPC_COUNTER_CLEAR_ON_CHAN0_FALL   0x01
#define LPC_COUNTER_CLEAR_ON_CHAN1_RISE   0x02
#define LPC_COUNTER_CLEAR_ON_CHAN1_FALL   0x03
#define LPC_COUNTER_CLEAR_ON_CHAN2_RISE   0x04
#define LPC_COUNTER_CLEAR_ON_CHAN2_FALL   0x05
#define LPC_COUNTER_CLEAR_ON_CHAN3_RISE   0x06
#define LPC_COUNTER_CLEAR_ON_CHAN3_FALL   0x07

/* PWM */
#define LPC_PWM_CHANNEL_ENABLE(x)    (0x01 << ((x) & 0x03))


#endif /* DRIVERS_COUNTER_TIMERS_H */


