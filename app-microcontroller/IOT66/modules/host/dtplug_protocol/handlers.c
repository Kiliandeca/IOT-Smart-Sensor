/****************************************************************************
 *  DTPlug serial communication protocol handler for tests.
 *
 *   handlers.c
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


/* This protocol handler is designed to run on a host replacing the DTPlug,
 * but should be easy to use as a source for the protocol handling code for
 * the DTPlug itself.
 */


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "dtplug_protocol_host.h"



int handle_module_data(struct line_transceiver* slave)
{
	struct header* head = &(slave->rx_packet.info);
	/* If this packet holds no valid data, it indicates that there are errors */
	if (head->seq_num & PACKET_IS_ERROR) {
		printf("Received a packet indicating errors (%d - %d)\n",
					 head->err.error_code, head->err.info);
		/* Request the list of errors */
		host_send_packet(slave, PKT_TYPE_GET_ERRORS, 0, NULL, 1);
		return 0;
	}
	/* Move data from "QUICK_DATA_PACKET" packets to the data part of the packet for easy handling */
	if (head->seq_num & QUICK_DATA_PACKET) {
		slave->rx_packet.data[0] = head->quick_data[0];
		slave->rx_packet.data[1] = head->quick_data[1];
		if (head->seq_num & QUICK_DATA_PACKET_ONE_BYTE) {
			head->data.size = 1;
		} else {
			head->data.size = 2;
		}
	}

	switch (head->type) {
		case PKT_TYPE_PING:
			break;

		case PKT_TYPE_GET_BOARD_INFO:
			break;

		case PKT_TYPE_GET_NUM_PACKETS:
			break;

		case PKT_TYPE_GET_ERRORS:
			/* FIXME */
			printf("Received error list: ... implement error decoding.\n");
			break;

		case PKT_TYPE_GET_NUM_ERRORS:
			/* FIXME */
			printf("Received number of errors: %d.\n", 0);
			break;
	
		case PKT_TYPE_CONTINUED_DATA:
			break;
	
		case PKT_TYPE_GET_TEMPERATURE:
			break;
	
		case PKT_TYPE_GET_ADC_VALUE:
			break;

		case PKT_TYPE_GET_GPIO:
			break;
	
		case PKT_TYPE_SEND_ON_BUS:
			break;
	
		default:
			printf("Received packet type %d. Not handled.\n", head->type);
			break;
	}
	return 0;
}



int handle_udp_request(char* buf, int len, struct sockaddr* addr, socklen_t addr_len)
{
}



