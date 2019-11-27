/****************************************************************************
 *   lib/protocol/dtplug/slave.c
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

#include "core/system.h"
#include "core/iap.h"
#include "core/user_information_block.h"
#include "lib/stdio.h"
#include "lib/string.h"
#include "drivers/serial.h"
#include "extdrv/status_led.h"
#include "lib/time.h"

#include "lib/protocols/dtplug/defs.h"
#include "lib/protocols/dtplug/slave.h"

/******************************************************************************/
/* DTPlug (or DomoTab, PC, ...) Communication */

#define DTPP_MAX_HANDLERS 2
static struct dtplug_protocol_handle* dtpp_handles[DTPP_MAX_HANDLERS] = {0};



static void dtplug_protocol_decode(struct dtplug_protocol_handle* handle, uint8_t c);

static void dtplug_protocol_decoder_0(uint8_t c)
{
	dtplug_protocol_decode(dtpp_handles[0], c);
}
static void dtplug_protocol_decoder_1(uint8_t c)
{
	dtplug_protocol_decode(dtpp_handles[1], c);
}


/* Setup the UART used for communication with the host / master (the module is slave) */
void dtplug_protocol_set_dtplug_comm_uart(uint8_t uart_num, struct dtplug_protocol_handle* handle)
{
	void* decoder = dtplug_protocol_decoder_0;

	/* Basic parameter checks */
	if (uart_num >= DTPP_MAX_HANDLERS) {
		return;
	}
	if (handle == NULL) {
		return;
	}

	/* Configure and register handle and configure uart */
	handle->rx_packet = handle->packets;
	handle->packet_ok = NULL;
	handle->done_with_old_packet = 1;
	handle->uart = uart_num;
	dtpp_handles[uart_num] = handle;
	switch (uart_num) {
		case 1:
			decoder = dtplug_protocol_decoder_1;
			break;
		case 0:
		default:
			break;
	}
	uart_on(uart_num, 115200, decoder);
}


/* Tell the receive and decode routine that the "handle->packet_ok" packet is no more in use and that
 *  we are ready to handle a new one.
 */
void dtplug_protocol_release_old_packet(struct dtplug_protocol_handle* handle)
{
	handle->packet_ok = NULL;
	handle->done_with_old_packet = 1;
}



/* When a packet has not been handled we must not count it as acknowledged
 * On the next ping request the master will then see wich packet caused the problem.
 */
void dtplug_protocol_add_error_to_list(struct dtplug_protocol_handle* handle, struct header* info, uint8_t error_code)
{
	if (handle->num_errors_stored < (DTPP_MAX_ERROR_STORED * 2)) {
		if (error_code == NO_ERROR) {
			error_code = ERROR_PKT_NOT_HANDLED;
		}
		handle->error_storage[handle->num_errors_stored++] = error_code;
		handle->error_storage[handle->num_errors_stored++] = (info->seq_num & SEQUENCE_MASK);
	}
}


/* Handle packet reception, including checksums */
/* 'sum' is used to sum all the received characters, and if the last byte of sum is 0 for each
 *   part (header and data) then the packet is valid.
 * 'full_size' is the size of the whole packet, including header, updated as soon as the header
 *   is checked and valid
 */
static void dtplug_protocol_decode(struct dtplug_protocol_handle* handle, uint8_t c)
{
	static uint8_t rx_ptr = 0;
	static uint8_t sum = 0;
	static uint8_t full_size = 0;
	static struct header* info = NULL;

	status_led(red_on);
	/* Do not start reception before receiving the packet start character */
	if ((rx_ptr == 0) && (c != FIRST_PACKET_CHAR)) {
		return;
	}

	/* Store the new byte in the packet */
	if (rx_ptr < sizeof(struct packet)) {
		((uint8_t*)handle->rx_packet)[rx_ptr++] = c;
		sum += c;
	} else {
		goto next_packet;
	}

	/* Is this packet valid ? (at end of header reception) */
	if (rx_ptr == sizeof(struct header)) {
		if (sum != 0) {
			goto next_packet;
		}
		/* Start the new checksum for data (if any) */
		sum = 0;

		info = (struct header*)handle->rx_packet;
		full_size = sizeof(struct header);
		if (!(info->seq_num & QUICK_DATA_PACKET)) {
			/* Do not care about big data packets here */
			full_size += (info->data.size & PKT_SIZE_MASK);
		}
	}

	/* Did we receive the whole packet ? */
	if (rx_ptr == full_size) {
		/* From here on, the packet is valid, we can provide some feedback */
		/* Check data checksum */
		if (!(info->seq_num & QUICK_DATA_PACKET) && (sum != info->data.checksum)) {
			dtplug_protocol_add_error_to_list(handle, info, ERROR_IN_DATA_CHECKSUM);
			handle->errors_count++;
			goto next_packet;
		}
		/* Warning, if we are still using the old packet there's a problem */
		if (handle->done_with_old_packet == 0) {
			/* FIXME : what to do then ? inform the master ? ignore the new packet ? */
			dtplug_protocol_add_error_to_list(handle, info, ERROR_LAST_PKT_IN_PROCESS);
			handle->errors_count++;
			goto next_packet;
		}
		/* Count received packets */
		handle->packet_count++;
		/* Mark packet as OK : switch pointers */
		handle->packet_ok = handle->rx_packet;
		handle->done_with_old_packet = 0;
		/* Switch our receiving buffer (do not overide the last received packet !) */
		if (handle->rx_packet == handle->packets) {
			handle->rx_packet = handle->packets + 1;
		} else {
			handle->rx_packet = handle->packets;
		}
		status_led(green_on);
		/* And get ready to receive the next packet */
		goto next_packet;
	}

	return;

next_packet:
#ifdef DEBUG
	if (handle->done_with_old_packet != 0) {
		uprintf(handle->uart, "Rx:%d, f:%d, cnt:%d\n", rx_ptr, full_size, handle->packet_count);
		if (rx_ptr >= sizeof(struct header)) {
			struct header* h = &(handle->rx_packet->info);
			uprintf(handle->uart, "P: type:%03d, seq:%d, e:%d, q:%d\n", h->type, h->seq_num,
								(h->seq_num & PACKET_IS_ERROR), (h->seq_num & QUICK_DATA_PACKET));
		}
	}
#endif
	/* Wether the packet was OK or not doesn't matter, go on for a new one :) */
	full_size = 0;
	rx_ptr = 0;
	sum = 0;
}

/* This function handle sending replies when requested by the host.
 * When there is an error but the host did not request a reply, store the error for future request.
 * When a reply is effectively sent, the PACKET_NEEDS_REPLY bit is removed from the sequence number so the
 *   packet handling code will know if there is still a PING request to be answered.
 */
void dtplug_protocol_send_reply(struct dtplug_protocol_handle* handle,
									struct packet* question, uint8_t error, int size, uint8_t* data)
{
	struct packet reply;
	struct header* tx_info = &(reply.info);
	int i = 0, sent = 0, len = 0;
	uint8_t sum = 0;
	int data_send_size = size, data_sent = 0;
	uint8_t* data_src = data;
	uint8_t type = question->info.type;

	if (error != NO_ERROR) {
		handle->errors_count++;
	}
	/* If no reply requested : we were called thus there have been an error. We should store the error and return. */
	if (!(question->info.seq_num & PACKET_NEEDS_REPLY)) {
		/* If we still have some room for the error, then keep track of it (if any),
		 * otherwise ... drop it, we don't have that much memory to keep track of hundreds of errors */
		if (error != NO_ERROR) {
			dtplug_protocol_add_error_to_list(handle, &(question->info), error);
		}
		return;
	}

	/* Remove the PACKET_NEEDS_REPLY bit as the reply is being built to prevent multiple replies to the
	 *   same packet. (do it now to prevent the mask when building the type field of the reply) */
	question->info.seq_num &= ~(PACKET_NEEDS_REPLY);

	/* If any error stored, send "got many errors" reply and store the error, but if
	 *    the host is reading the error table, no need to say there is an error table.
	 *    Rather send any possible new error.
	 */
	if (handle->num_errors_stored != 0) {
		/* Store the new error if any */
		if (error != NO_ERROR) {
			dtplug_protocol_add_error_to_list(handle, &(question->info), error);
		}
		/* The master wants to get all errors, give them even if there is an error in the sequence number */
		if (question->info.type == PKT_TYPE_GET_ERRORS) {
			data_send_size = handle->num_errors_stored;
			size = handle->num_errors_stored;
			data_src = handle->error_storage;
			handle->num_errors_stored = 0;
		} else {
			error = GOT_MANY_ERRORS;
			data_send_size = 0;
			size = 0;
		}
	}

	do {
		/* Does the data fit in the message ? */
		if (data_send_size > PACKET_DATA_SIZE) {
			data_send_size = PACKET_DATA_SIZE;
		}

		/* Build the reply */
		tx_info->start = FIRST_PACKET_CHAR;
		tx_info->type = type;
		tx_info->seq_num = question->info.seq_num;

		len = sizeof(struct header); /* At least, send header on serial link */
		if (error) {
			tx_info->seq_num |= PACKET_IS_ERROR;
			tx_info->err.error_code = error;
			if (error == GOT_MANY_ERRORS) {
				tx_info->err.info = handle->num_errors_stored;
			}
		} else {
			/* Append possible data */
			if ((data_src != NULL) && (data_send_size != 0)) {
				/* Can we send a quick data packet ? */
				if (size && (size <= 2)) {
					tx_info->seq_num |= QUICK_DATA_PACKET;
					tx_info->quick_data[0] = data_src[0];
					tx_info->quick_data[1] = data_src[1];
				} else {
					/* Copy data, compute checksum (also OK for a data_send_size of 0) */
					sum = 0;
					for (i = 0; i < data_send_size; i++) {
						reply.data[i] = data_src[i];
						sum += data_src[i]; /* Build checksum */
					}
					/* And update header information */
					tx_info->data.size = data_send_size;
					/* Will this packet be continued in the following one ? */
					if (data_send_size < (size - data_sent)) {
						tx_info->data.size |= BIG_DATA_PKT;
					}
					tx_info->data.checksum = sum;
					/* Update length of data to send on serial link */
					len += data_send_size;
				}
			}
		}

		/* Compute header checksum */
		sum = 0;
		tx_info->checksum = 0;
		for (i = 0; i < sizeof(struct header); i++) {
			sum += ((uint8_t*)tx_info)[i];
		}
		tx_info->checksum = ((uint8_t)(256 - sum));

		/* And send the reply */
		sent = 0;
		while (sent < len) {
			int ret = serial_write(handle->uart, (((char*)(&reply)) + sent), (len - sent));
			if (ret >= 0) {
				sent += ret;
			} else {
				/* Store a sending error, though it may never be sent ... */
				dtplug_protocol_add_error_to_list(handle, &(question->info), ERROR_IN_UART_TX);
				handle->errors_count++;
				return;
			}
			/* The serial baud rate is 115200, which means 64 bytes are sent in a little less than 6 ms.
			 * When there is not enougth place in the buffer, the return value should be 64, meaning 64 byte
			 *   were stored for transmission. Sleep for 6ms and try sending the next part.
			 */
			if (sent < len) {
				msleep(5);
			}
		}
		data_sent += data_send_size;

		/* Need to send more ? */
		if (data_sent < size) {
			/* Move data pointer */
			data_src += data_send_size;
			/* Update size to send. check against PACKET_DATA_SIZE is done at beginning of loop */
			data_send_size = (size - data_sent);
			/* Set packet type to continued data packet for following packets */
			type = PKT_TYPE_CONTINUED_DATA;
		}
	} while (data_sent < size);
}


/******************************************************************************/
/* User information block re-programming
 * Make sure that data is aligned on 4 bytes boundary, and that size is a multiple of 4.
 */
static int dtplug_protocol_user_flash_update(struct dtplug_protocol_handle* handle,
											 struct packet* question, void* data, int size)
{
	int ret = 0;
	/* Erase the user flash information pages */
	ret = iap_erase_info_page(0, 2);
	if (ret != 0) {
		dtplug_protocol_send_reply(handle, question, ERROR_FLASH_ERASE, 0, NULL);
		return -1;
	}
	ret = iap_copy_ram_to_flash((uint32_t)get_user_info(), (uint32_t)data, size);
	if (ret != 0) {
		dtplug_protocol_send_reply(handle, question, ERROR_FLASH_WRITE, 0, NULL);
		return -1;
	}
	return 0;
}


/******************************************************************************/
/* Common packets handlers.
 * Return 1 when packet has been handled, or 0 if the type is not a common one and some
 *   board or application specific code should take care of it.
 */
static int dtplug_protocol_common_handles(struct dtplug_protocol_handle* handle, struct packet* question)
{
	uint32_t tmp_val_swap = 0;
	/* These we can always handle */
	switch (question->info.type) {
		case PKT_TYPE_PING:
			question->info.seq_num |= PACKET_NEEDS_REPLY; /* Make sure the reply will be sent */
			dtplug_protocol_send_reply(handle, question, NO_ERROR, 0, NULL); /* A ping needs no aditional data */
			dtplug_protocol_release_old_packet(handle);
			break;
		case PKT_TYPE_RESET:
			/* Software reset of the board. No way out. */
			NVIC_SystemReset();
			break;
		case PKT_TYPE_SET_TIME:
			{
				struct time_spec new_time, time_diff;
				uint32_t* seconds = (uint32_t*)question->data;
				uint16_t* msec = (uint16_t*)&(question->data[4]);
				uint8_t time_buff[6];
				if (question->info.seq_num & QUICK_DATA_PACKET) {
					dtplug_protocol_send_reply(handle, question, ERROR_IN_PKT_STRUCTURE, 0, NULL);
					break;
				}
				if (question->info.data.size != 6) {
					dtplug_protocol_send_reply(handle, question, ERROR_IN_DATA_VALUES, 0, NULL);
					break;
				}
				new_time.seconds = byte_swap_32(*seconds);
				new_time.msec = (uint16_t)byte_swap_16(*msec);
				if (!(question->info.seq_num & PACKET_NEEDS_REPLY)) {
					set_time(&new_time);
				} else {
					set_time_and_get_difference(&new_time, &time_diff);
					time_to_buff_swapped(time_buff, &time_diff);
					dtplug_protocol_send_reply(handle, question, NO_ERROR, 6, time_buff);
				}
			}
			dtplug_protocol_release_old_packet(handle);
			break;
		case PKT_TYPE_SET_USER_INFO:
			{
				uint8_t tmp_data[sizeof(struct user_info)] __attribute__ ((__aligned__(4))) = {};
				uint8_t offset = question->data[0];
				uint8_t size = question->data[1];
				if (question->info.seq_num & QUICK_DATA_PACKET) {
					dtplug_protocol_send_reply(handle, question, ERROR_IN_PKT_STRUCTURE, 0, NULL);
					break;
				}
				/* Check that amount of data provided is OK and does not go beyond user_info structure end */
				if ((question->info.data.size != (size + 2)) || ((offset + size) > sizeof(struct user_info))) {
					dtplug_protocol_send_reply(handle, question, ERROR_IN_DATA_VALUES, 0, NULL);
					break;
				}
				/* Copy all board data before flash erase */
				memcpy(tmp_data, get_user_info(), sizeof(struct user_info));
				/* Update information in the copy */
				memcpy(&(tmp_data[offset]), &(question->data[2]), size);
				/* Update the user flash information pages */
				if (dtplug_protocol_user_flash_update(handle, question, tmp_data, sizeof(struct user_info)) != 0) {
					/* Reply got sent, if return value is not 0 */
					break;
				}
			}
			/* Software reset of the board. No way out. */
			NVIC_SystemReset();
			break;
		case PKT_TYPE_GET_NUM_PACKETS:
			question->info.seq_num |= PACKET_NEEDS_REPLY; /* Make sure the reply will be sent */
			tmp_val_swap = byte_swap_32(handle->packet_count);
			dtplug_protocol_send_reply(handle, question, NO_ERROR, 4, (uint8_t*)(&tmp_val_swap));
			dtplug_protocol_release_old_packet(handle);
			break;
		case PKT_TYPE_GET_ERRORS:
			question->info.seq_num |= PACKET_NEEDS_REPLY; /* Make sure the reply will be sent */
			dtplug_protocol_send_reply(handle, question, NO_ERROR, 0, NULL); /* Error handling code will take care of filling the message */
			dtplug_protocol_release_old_packet(handle);
			break;
		case PKT_TYPE_GET_NUM_ERRORS:
			question->info.seq_num |= PACKET_NEEDS_REPLY; /* Make sure the reply will be sent */
			tmp_val_swap = byte_swap_32(handle->errors_count);
			dtplug_protocol_send_reply(handle, question, NO_ERROR, 4, (uint8_t*)(&tmp_val_swap));
			dtplug_protocol_release_old_packet(handle);
			break;
		default:
			/* We do not handle this type, it must be a board specific one */
			return 0;
	}
	/* Packet handled */
	status_led(green_off);
	return 1;
}


/* Get a pointer to the new packet received.
 * Return NULL when no new packet were received since last packet was released.
 * If a new packet is present, call the common handles first.
 */
struct packet* dtplug_protocol_get_next_packet_ok(struct dtplug_protocol_handle* handle)
{
	if (handle->packet_ok != NULL) {
		struct packet* pkt_tmp = (struct packet*)handle->packet_ok;
		if (dtplug_protocol_common_handles(handle, pkt_tmp) == 0) {
			return pkt_tmp;
		}
	}
	return NULL;
}

