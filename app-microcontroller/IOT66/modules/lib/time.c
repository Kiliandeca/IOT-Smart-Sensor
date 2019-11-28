/****************************************************************************
 *   lib/time.c
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

#include "core/system.h"
#include "core/systick.h"
#include "lib/time.h"
#include "lib/string.h"


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

static volatile struct time_spec time = { 0, 0, };
static volatile uint32_t time_lock = 0;;

/* Interupt routine which keeps track of the time */
void time_track(uint32_t ms)
{
    /* This lock may have us miss one ms when time is changed, but this is perfectly OK, time is
     *    being changed !
     * Anyway, we are in interrupt context, we MUST NOT loop or sleep !
     */
    if (sync_lock_test_and_set(&time_lock, 1) == 1) {
        return;
    }
    time.msec++;
    if (time.msec >= 1000) {
        time.msec = 0;
        time.seconds++;
    }
    sync_lock_release(&time_lock);
}

/* Call this to set the time from a known good time source. */
void set_time(struct time_spec* new_time)
{
    /* We are not in interrupt context, we can wait for the lock to be released */
    while (sync_lock_test_and_set(&time_lock, 1) == 1) {};
    time.seconds = new_time->seconds;
    time.msec = new_time->msec;
    sync_lock_release(&time_lock);
}

/* Call this to set the time from a known good time source.
 * This function returns the time difference in the given time_spec.
 */
void set_time_and_get_difference(struct time_spec* new_time, struct time_spec* diff)
{
	struct time_spec tmp_old;
	if (new_time == NULL) {
		return;
	}
    /* We are not in interrupt context, we can wait for the lock to be released */
    while (sync_lock_test_and_set(&time_lock, 1) == 1) {};
	/* Save the old time */
	tmp_old.seconds = time.seconds;
	tmp_old.msec = time.msec;
	/* Set the time as soon as possible */
    time.seconds = new_time->seconds;
    time.msec = new_time->msec;
    sync_lock_release(&time_lock);

	/* And now compute the time difference */
	if (diff == NULL) {
		return;
	}
	if (new_time->seconds == tmp_old.seconds) {
		diff->msec = new_time->msec - tmp_old.msec;
	} else {
		diff->seconds = new_time->seconds - tmp_old.seconds;
		if (new_time->seconds > tmp_old.seconds) {
			diff->msec = (1000 - tmp_old.msec) + new_time->msec;
		} else {
			diff->msec = (1000 - new_time->msec) + tmp_old.msec;
		}
		if (diff->msec > 1000) {
			diff->msec -= 1000;
		} else {
			diff->seconds -= 1;
		}
	}
}


/* Put a time struct in a buffer, swapping both fields to network endian. */
void time_to_buff_swapped(uint8_t* buf, struct time_spec* src_time)
{
	struct time_spec time; /* Warning, this one will hold time in network endian ! */
	time.seconds = byte_swap_32(src_time->seconds);
	time.msec = (uint16_t)byte_swap_16(src_time->msec);
	memcpy(buf, &(time.seconds), 4);
	memcpy((buf + 4), &(time.msec), 2);
}


/* Get a snapshot of the time when this is called.
 * It is unsafe to call this one when in interrupt, use get_time_in_interrupt()
 */
void get_time(struct time_spec* save_time)
{
	/* FIXME : Check that we are not in interrupt ... */
	if (get_priority_mask() == 0) {
		get_time_in_interrupt(save_time);
		return;
	}
    /* We are not in interrupt context, we can wait for the lock to be released */
    while (sync_lock_test_and_set(&time_lock, 1) == 1) {};
	save_time->seconds = time.seconds;
	save_time->msec = time.msec;
    sync_lock_release(&time_lock);
}

/* Get a snapshot of the time when this is called
 * It is safe to call this one in interrupt.
 */
void get_time_in_interrupt(struct time_spec* save_time)
{
    /* We are in interrupt context, we can't wait for the lock to be released */
	save_time->seconds = time.seconds;
	save_time->msec = time.msec;
}

/* Must be called once to register the systick callback. */
static uint8_t time_configured = 0;
void time_init(void)
{
	if (time_configured != 0) {
		return;
	}
	time_configured = 1;
	add_systick_callback(time_track, 1); /* callback, period (ms) */
}


/* Compute a time difference
 * Return 0 if both times are the same, 1 if (t1 > t2), -1 if (t1 < t2)
 */
int get_time_diff(const struct time_spec* t1, const struct time_spec* t2, struct time_spec* diff)
{
	if (t1->seconds < t2->seconds) {
		diff->seconds = (t2->seconds - t1->seconds - 1);
		diff->msec = ((t2->msec + 1000) - t1->msec);
		if (diff->msec >= 1000) {
			diff->msec -= 1000;
			diff->seconds++;
		}
		return -1;
	} else if (t1->seconds == t2->seconds) {
		diff->seconds = 0;
		if (t1->msec < t2->msec) {
			diff->msec = t2->msec - t1->msec;
			return -1;
		} else if (t1->msec == t2->msec) {
			diff->msec = 0;
			return 0;
		} else {
			diff->msec = t1->msec - t2->msec;
			return 1;
		}
	} else {
		diff->seconds = (t1->seconds - t2->seconds - 1);
		diff->msec = ((t1->msec + 1000) - t2->msec);
		if (diff->msec >= 1000) {
			diff->msec -= 1000;
			diff->seconds++;
		}
		return 1;
	}
}


