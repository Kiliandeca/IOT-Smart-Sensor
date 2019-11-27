/****************************************************************************
 *  LightHouse UDP to RF Sub1GHz module bridge.
 *
 *   handlers.c
 *
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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


/* This protocol handler is designed to run on a host. */


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "list.h"
#include "dtplug_protocol_host.h"


#define BUFF_LEN 60

struct info_data {
	struct list_head head;
	int type;  /* Information type */
	int addr;  /* Source address */
	int function;  /* Information function */
	int number;    /* Information number */
	int value; /* Information */
	struct timeval last_update;   /* Information timestamp */
};


struct list_head* all_info = NULL;
int nb_info = 0;

int create_info(int type, int addr, int func, int num)
{
	struct info_data* addinfo = malloc(sizeof(struct info_data));

	if (addinfo == NULL) {
		return -1;
	}

	if (all_info == NULL) {
		all_info = &(addinfo->head);
		INIT_LIST_HEAD(&(addinfo->head));
	} else {
		list_add_tail(&addinfo->head, all_info);
	}
	addinfo->type = type;
	addinfo->addr = addr;
	addinfo->function = func;
	addinfo->number = num;
	nb_info++;

	return 0;
}

void delete_info(struct info_data* oldinfo)
{
	if (oldinfo == NULL) {
		return;
	}
	list_del(&(oldinfo->head));
	if (list_empty(all_info)) {
		all_info = NULL;
	}
	free(oldinfo);
	nb_info--;
}

/* Find exactly matching information */
struct info_data* find_info(int type, int addr, int func, int num)
{
	struct info_data* info = NULL;
	if (all_info == NULL) {
		return NULL;
	}
	list_for_each_entry(info, all_info, head) {
		if ((info->type == type) && (info->addr == addr) && (info->function == func) && (info->number == num)) {
			return info;
		}
	}
	return info;
}
/* TOTO : find all info_data matching given criteria */



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
			{
				struct info_data* info = find_info('T', 0, 0, 0); /* FIXME */
				uint16_t* tmp = (uint16_t*)(&(slave->rx_packet.data[0]));
				gettimeofday(&(info->last_update), NULL);
				info->value = (int16_t)ntohs(tmp[0]);
			}
			break;
	
		case PKT_TYPE_GET_ADC_VALUE:
			break;

		default:
			printf("Received packet type %d. Not handled.\n", head->type);
			break;
	}
	return 0;
}

#define REP_SIZE_MAX  1024

int handle_udp_request(char* buf, int len, struct sockaddr* addr, socklen_t addr_len,
						 int sock, struct line_transceiver* slave)
{
	struct info_data* info = NULL;
	if ((len == 0) || (buf == NULL)) {
		return -1;
	}
	if (buf[0] == 'G') {
		char obuf[REP_SIZE_MAX];
		int len = 0;
		/* GET command : Return the last updated value with the update timestamp */
		switch (buf[1]) {
			case 'L':
				info = find_info('L', 0, 0, 0); /* FIXME */
				if (info != NULL) {
					len = snprintf(obuf, REP_SIZE_MAX, "GL %d %d %d : %d at %ld.%03ld",
							info->addr, info->function, info->number, info->value,
							info->last_update.tv_sec, (info->last_update.tv_usec / 1000));
				}
				break;
			case 'I':
				list_for_each_entry(info, all_info, head) {
					if (info->type == 'I') {
						len = snprintf(obuf, REP_SIZE_MAX, "GI: %d %d %d\n",
							info->addr, info->function, info->value);
						sendto(sock, obuf, len, 0, addr, addr_len);
					}
				}
				break;
		}
		sendto(sock, obuf, len, 0, addr, addr_len);
		printf("Handled GET request type: %c\n", buf[1]);
	} else if (buf[0] == 'U') {
		uint8_t num = 0;
		/* Request a value update */
		switch (buf[1]) {
			case 'T':
				host_send_packet(slave, PKT_TYPE_START_TEMP_CONVERSION, 0, NULL, 0);
				usleep(200 * 1000); /* Temp conversion at 11 bits resolution is about 160ms */
				host_send_packet(slave, PKT_TYPE_GET_TEMPERATURE, 0, NULL, 1);
				break;
			case 'A':
				num = (uint8_t)(strtoul(&buf[2], NULL, 10) & 0xFF);
				host_send_packet(slave, PKT_TYPE_GET_ADC_VALUE, 1, &num, 1);
				break;
		}
		printf("Handled UPDATE request type: %c\n", buf[1]);
	} else if (buf[0] == 'S') {
		/* Send new values to the micro-controller */
		uint8_t obuf[BUFF_LEN];
		char* tmp = NULL;
		int i = 0;

		switch (buf[1]) {
			case 'L': /* FIXME */
				obuf[0] = (strtoul(&buf[2], &tmp, 10) & 0xFF); /* Address */
				obuf[1] = (strtoul(tmp, &tmp, 10) & 0x3F); /* Function */
				for (i = 2; ((i < (BUFF_LEN - 4)) && (tmp < buf + len)); i += 4) {
					obuf[i] = (strtoul(tmp, &tmp, 10) & 0xFF); /* Number */
					obuf[i + 1] = (strtoul(tmp, &tmp, 10) & 0xFF); /* Red Value */
					obuf[i + 2] = (strtoul(tmp, &tmp, 10) & 0xFF); /* Green Value */
					obuf[i + 3] = (strtoul(tmp, &tmp, 10) & 0xFF); /* Blue Value */
				}
				host_send_packet(slave, PKT_TYPE_SET_RGB_LED, i, obuf, 0);
				break;
		}
		printf("Handled SET request type: %c\n", buf[1]);
	} else {
		printf("Unhandled request type: '0x%02x'\n", buf[0]);
		return -2;
	}

	return 0;
}



