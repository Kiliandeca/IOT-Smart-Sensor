/****************************************************************************
 *   dtplug_protocol_host.c
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

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "dtplug_protocol_host.h"


/******************************************************************************/
/* Handle packet reception, including checksums */
/* 'sum' is used to sum all the received characters, and if the last byte of sum is 0 for each
 *   part (header and data) then the packet is valid.
 * 'full_size' is the size of the whole packet, including header, updated as soon as the header
 *   is checked and valid
 *
 * This function must be called for every received character.
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
int dtplug_protocol_decode(uint8_t c, struct line_transceiver* rx_data)
{
	struct header* info = (struct header*)(&(rx_data->rx_packet));
	int ret = 0;

	/* Do not start reception before receiving the packet start character */
	if ((rx_data->rx_ptr == 0) && (c != FIRST_PACKET_CHAR)) {
		return -1;
	}

	/* Store the new byte in the packet */
	((uint8_t*)(&(rx_data->rx_packet)))[rx_data->rx_ptr++] = c;
	rx_data->sum += c;

	/* Is this packet valid ? (at end of header reception) */
	if (rx_data->rx_ptr == sizeof(struct header)) {
		/* Checksum OK ? */
		if (rx_data->sum != 0) {
			rx_data->errors_count++;
			ret = -3;
			goto next_packet;
		}
		/* Start the new checksum for data (if any) */
		rx_data->sum = 0;

		rx_data->full_size = sizeof(struct header);
		/* If the packet is not a quick data or error packet it has a data part. Get it's size */
		if (!(info->seq_num & QUICK_DATA_PACKET) && !(info->seq_num & PACKET_IS_ERROR)) {
			rx_data->full_size += (info->data.size & PKT_SIZE_MASK); /* Don't care about big packets here */
			/* Make sure the packet will fit in the buffer */
			if (rx_data->full_size > sizeof(struct packet)) {
				rx_data->errors_count++;
				ret = -4;
				goto next_packet;
			}
		}
	}

	/* Did we receive the whole packet ? */
	if (rx_data->rx_ptr == rx_data->full_size) {
		/* From here on, the packet is valid, we can provide some feedback */
		/* Check data checksum if we have a normal data packet */
		if (!(info->seq_num & QUICK_DATA_PACKET) && !(info->seq_num & PACKET_IS_ERROR)) {
			if (rx_data->sum != info->data.checksum) {
				rx_data->errors_count++;
				ret = -2;
				goto next_packet;
			}
		}
		/* Count received packets */
		rx_data->packet_rx_count++;
		ret = rx_data->full_size;
		/* And get ready to receive the next packet */
		goto next_packet;
	}

	return 0;

next_packet:
#ifdef DEBUG
	printf("Current rx pointer: %d, packet full size: %d\n", rx_data->rx_ptr, rx_data->full_size);
	printf("Packets received count: %d, packets errors: %d\n",
				rx_data->packet_rx_count, rx_data->errors_count);
	if (rx_data->rx_ptr >= sizeof(struct header)) {
		struct header* h = info;
		printf("Pkt: type: %d, seq: %d, err: %d, quick: %d\n", h->type, h->seq_num,
						(h->seq_num & PACKET_IS_ERROR), (h->seq_num & QUICK_DATA_PACKET));
	}
#endif
	/* Wether the packet was OK or not doesn't matter, go on for a new one :) */
	rx_data->full_size = 0;
	rx_data->rx_ptr = 0;
	rx_data->sum = 0;
	return ret;
}


/******************************************************************************/
/* This function handles sending packets to slave
 * It returns the number of bytes of data sent.
 */
int host_send_packet(struct line_transceiver* slave, uint8_t type, uint32_t size, uint8_t* data, int need_reply)
{
	struct packet cmd;
	struct header* cmd_info = &(cmd.info);
	unsigned int i = 0, sent = 0, len = sizeof(struct header);
	uint8_t sum = 0;
	unsigned int data_send_size = size, data_sent = 0;

	/* Loop to send all data, possibly split in several packets */
	do {
		if (data_send_size > PACKET_DATA_SIZE) {
			data_send_size = PACKET_DATA_SIZE;
		}
		/* Set packet header */
		cmd_info->start = FIRST_PACKET_CHAR;
		cmd_info->type = type;
		cmd_info->seq_num = (slave->sequence_number++ & SEQUENCE_MASK);
		/* Set the need reply bit only on the last packet */
		if ((need_reply != 0) && ((data_sent + data_send_size) == size)) {
			cmd_info->seq_num |= PACKET_NEEDS_REPLY;
		}
		/* Setup data */
		if (data != NULL) {
			/* Only use quick data packet for short packet, not continued data */
			if (size && (size <= 2)) {
				cmd_info->seq_num |= QUICK_DATA_PACKET;
				cmd_info->quick_data[0] = data[0];
				cmd_info->quick_data[1] = data[1];
			} else {
				/* Copy data, compute checksum (also OK for a data_send_size of 0) */
				for (i = 0; i < data_send_size; i++) {
					cmd.data[i] = data[i];
					sum += data[i]; /* Build checksum */
				}
				/* And update header information */
				cmd_info->data.size = data_send_size;
				/* Will this packet be continued in the following one ? */
				if (data_send_size < (size - data_sent)) {
					cmd_info->data.size |= BIG_DATA_PKT;
				}
				cmd_info->data.checksum = sum;
				/* Update length of data to send on serial link */
				len += data_send_size;
			}
		}

		/* Compute header checksum */
		sum = 0;
		cmd_info->checksum = 0;
		for (i = 0; i < sizeof(struct header); i++) {
			sum += ((uint8_t*)cmd_info)[i];
		}
		cmd_info->checksum = ((uint8_t)(256 - sum));

		/* And send the packet on the serial link */
		while (sent < len) {
			int ret = write(slave->fd, (((char*)(&cmd)) + sent), (len - sent));
			if (ret >= 0) {
				sent += ret;
			} else {
				/* Sending error ... */
				/* FIXME : handle / report errors */
				return data_sent;
			}
		}
		data_sent += data_send_size;

		/* Need to send more ? */
		if (data && (data_sent <= size)) {
			/* Move data pointer */
			data += data_send_size;
			/* Update size to send. check against PACKET_DATA_SIZE is done at beginning of loop */
			data_send_size = (size - data_sent);
			/* Set packet type to continued data packet for following packets */
			type = PKT_TYPE_CONTINUED_DATA;
			/* And prepare sending of the next packet (reset internal loop counters) */
			sum = 0;
			len = sizeof(struct header);
			sent = 0;
		} else {
			/* No data, everything got sent */
			data_send_size = 0;
		}
	} while (data_send_size != 0);

	return data_sent;
}


