/****************************************************************************
 *   lib/time.h
 *
 *
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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
 ****************************************************************************/

#ifndef LIB_TIME_H
#define LIB_TIME_H


#include "lib/stdint.h"


/******************************************************************************/
/* Common parts for time handling for LPC Boards with or without RTC */

/* Notes on time tracking using this library :
 *
 * - Unless there is some known good time source used to set time (using set_time()) the time is
 *     counted from the system power ON or RTC power ON as origin.
 *
 * - This code relies on systick timer configured with a 1ms period (systick_timer_on(1)) for whole
 *     time on systems without a RTC oscilator and on systick timer for mili-seconds only on systems
 *     with a 32.768 KHz external oscilator for the RTC.
 *
 * - When used with systick only, the time tracking is far from the precision time one should
 *     obtain using the RTC with an external 32.768 KHz cristal. It uses the 12MHz internal RC
 *     Oscilator, which is at 1% accuracy.
 *     Thus the time error may be as much as 1s every 100 seconds !
 */

struct time_spec {
	uint32_t seconds;
	uint16_t msec;
};


/* Call this to set the time from a known good time source. */
void set_time(struct time_spec* new_time);


/* Call this to set the time from a known good time source.
 * This function returns the time difference in the given time_spec.
 */
void set_time_and_get_difference(struct time_spec* new_time, struct time_spec* diff);


/* Put a time struct in a buffer, swapping both fields to network endian. */
void time_to_buff_swapped(uint8_t* buf, struct time_spec* src_time);


/* Get a snapshot of the time when this is called.
 * When in interrupt, use get_time_in_interrupt(). It will get called anyway, but it
 *   will be longer.
 */
void get_time(struct time_spec* save_time);

/* Get a snapshot of the time when this is called
 * It is safe to call this one in interrupt.
 */
void get_time_in_interrupt(struct time_spec* save_time);


/* Must be called once to register the systick callback.
 * Can be called anytime, will just return if it has already been called.
 */
void time_init(void);


/* Compute a time difference
 * Return 0 if both times are the same, 1 if (t1 > t2), -1 if (t1 < t2)
 */
int get_time_diff(const struct time_spec* t1, const struct time_spec* t2, struct time_spec* diff);



#endif /* LIB_TIME_H */

