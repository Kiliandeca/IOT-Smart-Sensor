/****************************************************************************
 *  DTPlug serial communication protocol test.
 *
 *   main.c
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include "dtplug_protocol_host.h"


#define SERIAL_BAUD  B115200

#define BUF_SIZE  256



int serial_setup(char* name)
{
	struct termios tio;
	int fd = -1;

	/* Open serial port */
	fd = open(name, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		perror("Unable to open communication with companion chip");
		return -1;
	}
	/* Setup serial port */
	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = CS8 | CREAD | CLOCAL;	/* 8n1, see termios.h for more information */
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 5;
	cfsetospeed(&tio, SERIAL_BAUD);
	cfsetispeed(&tio, SERIAL_BAUD);
	tcsetattr(fd, TCSANOW, &tio);

	return fd;
}



int main(int argc, char* argv[])
{
	int out_fd = 1;
	struct line_transceiver slave;
	struct timeval next_periodic_round;
	struct timeval period;

	/* Need Serial port and destination file as parameter */
	if (argc < 2) {
		printf("Need tty device number and optional destination file\n");
		printf("Please start with \"%s /dev/ttyUSB0 temp_log\" (for example)\n", argv[0]);
		return -1;
	}

	memset(&slave, 0, sizeof(struct line_transceiver));
	/* Open tty */
	slave.fd = serial_setup(argv[1]);
	if (slave.fd < 0) {
		printf("Unable to open specified serial port %s\n", argv[1]);
		return -4;
	}

	if (argc == 3) {
		/* FIXME : log to file */
	}


	/* Periodic actions setup */
	gettimeofday(&next_periodic_round, NULL);
	period.tv_sec = 2;
	period.tv_usec = 0;
	timeradd(&next_periodic_round, &period, &next_periodic_round);


	/* ************************************************* */
	/* And never stop getting data ! */
	while (1) {
		struct timeval now;
		char c = 0;
		int ret = 0, len = 0, i = 0, nb_val = 0;
		char outbuff[BUF_SIZE];

		/* Send periodic requests for temperature */
		gettimeofday(&now, NULL);
		if (timercmp(&now, &next_periodic_round, >=)) {
			/* Send new requests */
			host_send_packet(&slave, PKT_TYPE_START_TEMP_CONVERSION, 0, NULL, 0);
			usleep(200 * 1000); /* Temp conversion at 11 bits resolution is about 160ms */
			host_send_packet(&slave, PKT_TYPE_GET_TEMPERATURE, 0, NULL, 1);
			usleep(5000);
			/* Setup next periodic request date */
			timeradd(&next_periodic_round, &period, &next_periodic_round);
		}

		/* Get serial data and try to build a packet */
		if (read(slave.fd, &c, 1) != 1) {
			usleep(100);
			continue;
		}
		ret = dtplug_protocol_decode(c, &slave);
		/* Check return code to know if we have a valid packet */
		if (ret == -1) {
			printf("%c", c);
			continue;
		}
		if (ret < 0) {
			printf("\nError in received packet. (ret: %d)\n", ret);
			/* FIXME : dump packet for debugging */
			continue;
		}
		if (ret == 0) {
			continue; /* Packet is being built */
		}

		if (slave.rx_packet.info.type == PKT_TYPE_GET_ERRORS) {
			printf("Received error list: ... implement error decoding.\n"); /* FIXME */
			continue;
		}
		if (slave.rx_packet.info.type != PKT_TYPE_GET_TEMPERATURE) {
			printf("Received unrequested packet.\n");
			continue;
		}
		if (slave.rx_packet.info.seq_num & PACKET_IS_ERROR) {
			/* This packet holds no valid data, it indicates that there are errors */
			printf("Received a packet indicating errors (%d - %d)\n",
						 slave.rx_packet.info.err.error_code, slave.rx_packet.info.err.info);
			/* Request the list of errors */
			host_send_packet(&slave, PKT_TYPE_GET_ERRORS, 0, NULL, 1);
			continue;
		}

		/* Get / Check data */
		if (slave.rx_packet.info.seq_num & QUICK_DATA_PACKET) {
			nb_val = 1;
			/* Move data to the data part of the packet for easy handling */
			slave.rx_packet.data[0] = slave.rx_packet.info.quick_data[0];
			slave.rx_packet.data[1] = slave.rx_packet.info.quick_data[1];
		} else {
			nb_val = (slave.rx_packet.info.data.size / 2);
		}

		/* Build log line */
		len = snprintf(outbuff, BUF_SIZE, "%d.%03d :", (int)now.tv_sec, (int)(now.tv_usec / 1000));
		for (i = 0; i < nb_val; i++) {
			int tmp = 0, val = 0;

			val = ntohs( ((uint16_t*)(&(slave.rx_packet.data)))[i] );
			tmp = snprintf((outbuff + len), (BUF_SIZE - len), " %d,%d", (val / 10), (val % 10));
			len += tmp;
		}
		len += snprintf((outbuff + len), (BUF_SIZE - len), "\n");
		/* And write to log */
		write(out_fd, outbuff, len);

	} /* End of infinite loop */

	close(slave.fd);
	close(out_fd);
	return 0;
}


