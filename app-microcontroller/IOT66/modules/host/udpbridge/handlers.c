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
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "dtplug_protocol_host.h"



#define PKT_TYPE_GET_DISTANCES    (PKT_TYPE_LAST + 40)
#define PKT_TYPE_GET_SMOKE        (PKT_TYPE_LAST + 41)
#define PKT_TYPE_GET_BATTERY      (PKT_TYPE_LAST + 42)
#define PKT_TYPE_GET_IR           (PKT_TYPE_LAST + 43)

#define PKT_TYPE_SET_SPEED        (PKT_TYPE_LAST + 50)
#define PKT_TYPE_SET_DIRECTION    (PKT_TYPE_LAST + 51)



int16_t temperature0 = 0;
int16_t temperature1 = 0;
struct timeval temp_timestamp;

uint16_t adc0 = 0;
uint16_t adc1 = 0;
struct timeval adc_timestamp;

uint16_t smoke = 0;
struct timeval smoke_timestamp;

#define NB_PULSE_SENSORS  6
uint16_t distances[NB_PULSE_SENSORS] = {0};
struct timeval distances_timestamp;


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
				uint16_t* tmp = (uint16_t*)(&(slave->rx_packet.data));
				gettimeofday(&temp_timestamp, NULL);
				temperature0 = (int16_t)ntohs(tmp[0]);
				temperature1 = (int16_t)ntohs(tmp[1]);
			}
			break;
	
		case PKT_TYPE_GET_ADC_VALUE:
			{
				uint16_t* tmp = (uint16_t*)(&(slave->rx_packet.data));
				gettimeofday(&adc_timestamp, NULL);
				adc0 = ntohs(tmp[0]);
				adc1 = ntohs(tmp[1]);
			}
			break;

		case PKT_TYPE_GET_SMOKE:
			{
				uint16_t* tmp = (uint16_t*)(&(slave->rx_packet.data));
				gettimeofday(&smoke_timestamp, NULL);
				smoke = ntohs(tmp[0]);
			}
			break;

		case PKT_TYPE_GET_DISTANCES:
			{
				uint16_t* tmp = (uint16_t*)(&(slave->rx_packet.data));
				int i = 0;
				gettimeofday(&distances_timestamp, NULL);
				for (i = 0; i < NB_PULSE_SENSORS; i++) {
					distances[i] = ntohs(tmp[i]);
				}
			}
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
	if ((len == 0) || (buf == NULL)) {
		return -1;
	}
	if (buf[0] == 'G') {
		char obuf[REP_SIZE_MAX];
		int len = 0;
		/* GET command : Return the last updated value with the update timestamp */
		switch (buf[1]) {
			case 'T':
				len = snprintf(obuf, REP_SIZE_MAX, "GT: %d, %d at %ld.%03ld",
							temperature0, temperature1,
							temp_timestamp.tv_sec, (temp_timestamp.tv_usec / 1000));
				break;
			case 'A':
				len = snprintf(obuf, REP_SIZE_MAX, "GA: %d, %d at %ld.%03ld",
							adc0, adc1, adc_timestamp.tv_sec, (adc_timestamp.tv_usec / 1000));
				break;
			case 'S':
				len = snprintf(obuf, REP_SIZE_MAX, "GS: %d at %ld.%03ld",
							smoke, smoke_timestamp.tv_sec, (smoke_timestamp.tv_usec / 1000));
				break;
			case 'D':
				len = snprintf(obuf, REP_SIZE_MAX, "GD: %d, %d, %d, %d, %d, %d at %ld.%03ld",
							distances[0], distances[1], distances[2],
							distances[3], distances[4], distances[5],
							distances_timestamp.tv_sec, (distances_timestamp.tv_usec / 1000));
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
			case 'S':
				host_send_packet(slave, PKT_TYPE_GET_SMOKE, 0, NULL, 1);
				break;
			case 'D':
				host_send_packet(slave, PKT_TYPE_GET_DISTANCES, 0, NULL, 1);
				break;
		}
		printf("Handled UPDATE request type: %c\n", buf[1]);
	} else if (buf[0] == 'S') {
		/* Send a new value to the micro-controller */
		uint8_t obuf[4] = {0, 0, 0, 0};
		char* tmp = NULL;

		switch (buf[1]) {
			case 'L':
				obuf[0] = (strtoul(&buf[2], &tmp, 10) & 0xFF);
				obuf[1] = (strtoul(tmp, &tmp, 10) & 0xFF);
				obuf[2] = (strtoul(tmp, &tmp, 10) & 0xFF);
				obuf[3] = (strtoul(tmp, &tmp, 10) & 0xFF);
				host_send_packet(slave, PKT_TYPE_SET_RGB_LED, 4, obuf, 0);
				break;
			case 'D': /* Direction */
				obuf[0] = (strtoul(&buf[2], NULL, 10) & 0xFF);
				host_send_packet(slave, PKT_TYPE_SET_DIRECTION, 1, obuf, 0);
				break;
			case 'S': /* Speed */
				obuf[0] = (strtoul(&buf[2], NULL, 10) & 0xFF);
				host_send_packet(slave, PKT_TYPE_SET_SPEED, 1, obuf, 0);
				break;
		}
		printf("Handled SET request type: %c\n", buf[1]);
	} else {
		printf("Unhandled request type: '0x%02x'\n", buf[0]);
		return -2;
	}

	return 0;
}



