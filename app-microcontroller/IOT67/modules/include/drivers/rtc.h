/****************************************************************************
 *  drivers/rtc.h
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

#ifndef DRIVERS_RTC_H
#define DRIVERS_RTC_H

/***************************************************************************** */
/*                RTC and RTC Interrupts                                    */
/***************************************************************************** */

/* The RTC is an integrated module of the LPC122x.
 * Refer to LPC12xx documentation (UM10441.pdf) for more information
 */

#include "lib/stdint.h"
#include "core/lpc_regs.h"


/* Return the number of RTC ticks from system power on.
 * This count is from power being present, even if resets occured.
 * The calls made during the first three seconds after RTC timer start will return 0. (See
 *    UM10441 section 16.6.1)
 */
uint32_t rtc_get_count(void);


/***************************************************************************** */
/*   RTC setup   */

/* In case someone wants the RTC to count something different from seconds
 * No need to call this function if you only want to count seconds, this is the default.
 * source selection values are defined below.
 * clk_div is only usefull when selected clock source is PCLK (peripheral clock). Use value
 *    betwween 1 and 255 included.
 */
void rtc_clk_src_select(uint8_t source, uint8_t clk_div);

/* Start the RTC. Once the RTC has been started using this call, it cannot be stopped. */
void rtc_on(void);

/* Disable the RTC control block. Note that once started from software the RTC cannot be stopped. */
void rtc_ctrl_off(void);

/* This will disable the RTC. This is only possible if the RTC has not been started before. */
void rtc_disable(void);

/* Change the RTC value */
void rtc_set_date(uint32_t date);


/***************************************************************************** */
/* Set the match value for the RTC
 * If periodic is 1 then the rtc match register will be updated after the interrupt
 *    has been handled by adding the offset.
 * If date is set, the match register will be set to date. (regardless of the periodic
 *    argument, which allows to have the first interrupt at a given date, and the following
 *    ones at a period set by the offset argument.
 * If date is 0, then the match register is set to current date + offset.
 * return -1 if RTC is not started and synced.
 */
int rtc_set_match(uint32_t date, uint32_t offset, uint8_t periodic);

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
#define RTC_CB_INST_RETRY_MS  500
int set_rtc_callback(void (*callback) (uint32_t), uint32_t when, uint32_t period);
void remove_rtc_callback();



/***************************************************************************** */
/*                    Cortex-M0 RTC (Real-Time Clock)                          */
/***************************************************************************** */
/* Cortex-M0 RTC Registers */
struct lpc_rtc {
    volatile uint32_t data;     /* 0x000 : Data register (R/-) */
    volatile uint32_t match;    /* 0x004 : Match register (R/W) */
    volatile uint32_t load;     /* 0x008 : Load register (R/W) */
    volatile uint32_t control;  /* 0x00C : Control register (R/W) */
    volatile uint32_t intr_control;        /* 0x010 : Interrupt control set/clear register (R/W) */
    volatile uint32_t raw_intr_status;     /* 0x014 : Raw interrupt status register (R/-) */
    volatile uint32_t masked_intr_status;  /* 0x018 : Masked interrupt status register (R/-) */
    volatile uint32_t intr_clear;          /* 0x01C : Interrupt clear register (-/W) */
};
#define LPC_RTC  ((struct lpc_rtc*) LPC_RTC_BASE) /* SysTick configuration struct */

/* RTC Clock source selection */
#define LPC_RTC_CLK_1HZ          (0)
#define LPC_RTC_CLK_1HZ_DELAYED  (0x01)
#define LPC_RTC_CLK_1KHZ         (0x0A)
#define LPC_RTC_CLK_PCLK         (0x04)  /* Main clock divided by RTC clock divider value */

/* RTC control register */
#define LPC_RTC_START  (1UL << 0)
#define LPC_RTC_DISABLE  (0)

/* RTC interrupt control register */
#define LPC_RTC_INT_ENABLE  (1UL << 0)
#define LPC_RTC_INT_DISABLE  (0)

/* RTC interrupt clear register */
#define LPC_RTC_CLEAR_INTR  (1UL << 0)


#endif  /* DRIVERS_RTC_H */

