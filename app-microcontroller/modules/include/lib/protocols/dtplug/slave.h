/*
 * lib/protocols/dtplug/slave.h
 *
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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
 */

#ifndef LIB_PROTOCOLS_DTPLUG_SLAVE_H
#define LIB_PROTOCOLS_DTPLUG_SLAVE_H


#include "lib/stdint.h"
#include "lib/protocols/dtplug/defs.h"


/******************************************************************************/
/* DTPlug (or DomoTab, PC, ...) Communication */

#define DTPP_MAX_ERROR_STORED 8

struct dtplug_protocol_handle {
	/* Store two packets, one being received, one being used */
	struct packet packets[2];
	struct packet* rx_packet;
	volatile struct packet* packet_ok;

	uint32_t packet_count;

	uint32_t errors_count;
	uint8_t error_storage[(DTPP_MAX_ERROR_STORED * 2)];
	uint8_t num_errors_stored;

	/* Set to 1 when the packet is handled to tell the decoder we can handle a new one.
	 * MUST be initialised to 1 or the handle will think we have a valid packet to handle upon
	 * system startup */
	uint8_t done_with_old_packet;
	uint8_t uart;
};


/* Setup the UART used for communication with the host / master (the module is slave) */
void dtplug_protocol_set_dtplug_comm_uart(uint8_t uart_num, struct dtplug_protocol_handle* handle);


/* Tell the receive routine that the "packet_ok" packet is no more in use and that
 *  we are ready to handle a new one */
void dtplug_protocol_release_old_packet(struct dtplug_protocol_handle* handle);


/* Get a pointer to the new packet received.
 * Return NULL when no new packet were received since last packet was released.
 */
struct packet* dtplug_protocol_get_next_packet_ok(struct dtplug_protocol_handle* handle);


/* When a packet has not been handled we must not count it as acknowledged
 * On the next ping request the master will then see wich packet caused the problem.
 */
void dtplug_protocol_add_error_to_list(struct dtplug_protocol_handle* handle, struct header* info, uint8_t error_code);


/* This function handle sending replies when requested by the host.
 * When there is an error but the host did not request a reply, this function stores the error for
 *    future request.
 * When a reply is effectively sent, the PACKET_NEEDS_REPLY bit is removed from the sequence field
 *   packet handling code will know if there is still a PING request to be answered.
 */
void dtplug_protocol_send_reply(struct dtplug_protocol_handle* handle,
									struct packet* question, uint8_t error, int size, uint8_t* data);



#endif /* LIB_PROTOCOLS_DTPLUG_SLAVE_H */
