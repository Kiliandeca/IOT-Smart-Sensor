/****************************************************************************
 *   dtplug_protocol_host.h
 *
 *
 * Copyright 2013-2015 Nathael Pajani <nathael.pajani@ed3l.fr>
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


/* Host side implementation of the DTPlug communication protocol */

#ifndef DTPLUG_PROTOCOL_HOST_H
#define DTPLUG_PROTOCOL_HOST_H


#include <stdint.h>
#include "dtplug_protocol_defs.h"


/******************************************************************************/
/* Handle packet reception, including checksums */
/* 'sum' is used to sum all the received characters, and if the last byte of sum is 0 for each
 *   part (header and data) then the packet is valid.
 * 'full_size' is the size of the whole packet, including header, updated as soon as the header
 *   is checked and valid
 */
struct line_transceiver {
	int fd;
	/* The packet being built */
	struct packet rx_packet;
	/* Sequence number for this line */
	uint8_t sequence_number;
	/* Packet building data */
	uint8_t rx_ptr;
	uint8_t sum;
	uint8_t full_size;
	/* packet counts */
	uint32_t packet_rx_count;
	uint32_t packet_tx_count;
	uint32_t errors_count;
};

/* This function must be called for every received character.
 * If the character is part of a packet but the packet is being built, then the function returns 0.
 * When the character is the last one of a valid packet, then the function returns the packet size
 *   and the packet in rx_data->rx_packet is valid.
 * If the character is the last one of a packet which has an invalid data checksum, this function
 *   returns -2 and the data is lost.
 * If the character is not part of a packet it returns -1. The character may then be part of
 *   a debug message (and displayed by the host), or any other kind of communication.
 * When a set of consecutive characters have been used to build a packet but the packet is
 *   not valid due to header error, then the function returns -3 (checksum error) or -4 (data size
 *   error). The data in rx_data->rx_packet is the received data but is not valid.
 *   The corresponding data size is always sizeof(struct header).
 */
int dtplug_protocol_decode(uint8_t c, struct line_transceiver* rx_data);

/* This function handles sending packets to slave
 * It returns the number of bytes of data sent.
 */
int host_send_packet(struct line_transceiver* slave, uint8_t type, uint32_t size,  uint8_t* data, int need_reply);


#endif /* DTPLUG_PROTOCOL_HOST_H */

