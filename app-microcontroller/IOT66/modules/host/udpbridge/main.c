/****************************************************************************
 *  DTPlug serial communication protocol handler for tests.
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


/* this protocol handler is designed to run on a host replacing the DTPlug,
 * but should be easy to use as a source for the protocol handling code for
 * the DTPlug itself.
 */


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include "dtplug_protocol_host.h"
#include "serial_utils.h"
#include "sock_utils.h"
#include "handlers.h"


#define BUF_SIZE  1024

#define PROG_NAME  "DTPlug protocol test bridge"
#define VERSION  "0.1"


void help(char *prog_name)
{
	fprintf(stderr, "---------------- "PROG_NAME" --------------------------------\n");
	fprintf(stderr, "Usage: %s [options]\n" \
		"  Available options:\n" \
		"  \t -p | --port : Start listening for messages on this port\n" \
		"  \t -d | --device : Serial device to use for serial communication with the module\n" \
		"  \t -h | --help : Print this help\n" \
		"  \t -v | --version : Print programm version\n" \
		"  All other arguments are data for the command, handled depending on the command.\n", prog_name);
	fprintf(stderr, "-----------------------------------------------------------------------\n");
}


int main(int argc, char* argv[])
{
	/* Serial */
	char* device = NULL;
	struct line_transceiver slave;
	/* TCP Server */
	int port = -1;
	int server_socket = -1;
	/* For select */
	struct timeval next_periodic_round;
	struct timeval period;
	fd_set read_fds;
	int max_fd = 0;

	while(1) {
		int option_index = 0;
		int c = 0;

		struct option long_options[] = {
			{"port", required_argument, 0, 'p'},
			{"device", required_argument, 0, 'd'},
			{"help", no_argument, 0, 'h'},
			{"version", no_argument, 0, 'v'},
			{0, 0, 0, 0}
		};

		c = getopt_long(argc, argv, "p:d:hv", long_options, &option_index);

		/* no more options to parse */
		if (c == -1) break;
		switch (c) {
			/* a, board address */
			case 'p':
				port = strtoul(optarg, NULL, 0);
				break;

			/* c, command */
			case 'd':
				device = optarg;
				break;

			/* v, version */
			case 'v':
				printf("%s Version %s\n", PROG_NAME, VERSION);
				return 0;
				break;

			/* h, help */
			case 'h':
			default:
				help(argv[0]);
				return 0;
		}
	}


	/* Need Serial port and destination file as parameter */
	if ((port == -1) || (device == NULL)) {
		printf("Error, need both serial (tty) device and IP port number\n");
		help(argv[0]);
		return -1;
	}

	/* Open tty */
	memset(&slave, 0, sizeof(struct line_transceiver));
	slave.fd = serial_setup(device);
	if (slave.fd < 0) {
		printf("Unable to open specified serial port %s\n", device);
		return -2;
	}

	/* Create UDP server socket */
	server_socket = create_bound_udp_socket(port, NULL);
	if (server_socket <= 0) {
		printf("Unable to open the server UDP socket on port %d\n", port);
		return -3;
	}

	/* ************************************************* */
	/* Initial FD set setup */
	FD_ZERO(&read_fds);
	FD_SET(STDIN_FILENO, &read_fds);   /* Add stdin */
	FD_SET(slave.fd, &read_fds);  /* Serial link */
	FD_SET(server_socket, &read_fds);  /* New connexions */
	max_fd = server_socket + 1;  /* No close, this one is the last open, so it's the higest */

	/* Periodic actions setup */
	gettimeofday(&next_periodic_round, NULL);
	period.tv_sec = 1; /* "Timer" granularitu is 1 second */
	period.tv_usec = 0;
	timeradd(&next_periodic_round, &period, &next_periodic_round);


	/* ************************************************* */
	/* And never stop handling data ! */
	while (1) {
		int nb = 0, len = 0, ret = 0;
		struct timeval timeout;
		struct timeval now;
		fd_set tmp_read_fds = read_fds;
		char buf[BUF_SIZE];

		/* Send periodic requests for temperature */
		gettimeofday(&now, NULL);
		
		if (timercmp(&now, &next_periodic_round, >=)) {
			timeout.tv_sec = 0;
			timeout.tv_usec = 0;
		} else {
			timersub(&next_periodic_round, &now, &timeout);
		}

		/* select() call .... be kind to other processes */
		nb = select(max_fd, &tmp_read_fds, NULL, NULL, &timeout);
		/* Errors here are bad ones .. exit ?? */
		if (nb < 0) {
			perror ("select failed");
			printf("Error: select failed, this is critical.\n");
			return -10;
		}
		/* In case of timeout ... perform periodic actions if a period has been defined */
		if (nb == 0) {
			int misses = 0;
			/* Setup next periodic request date */
			timeradd(&next_periodic_round, &period, &next_periodic_round);
			gettimeofday(&now, NULL);
			/* Did we miss some rounds ? */
			while (timercmp(&now, &next_periodic_round, >=)) {
				/* Count misses (and catch up) */
				timeradd(&next_periodic_round, &period, &next_periodic_round);
				misses++;
			}
			/* FIXME : parse the list of periodic actions for new packets */
			continue;
		}

		/* Data from Ethernet side */
		if (FD_ISSET(server_socket, &tmp_read_fds)) {
			struct sockaddr_in addr;
			socklen_t addr_len = sizeof(struct sockaddr_in);

			/* Receive the new data */
			memset(buf, 0, BUF_SIZE);
			len = recvfrom(server_socket, buf, BUF_SIZE, 0, (struct sockaddr *)&addr, &addr_len);
			if (len > 0) {
				handle_udp_request(buf, len, (struct sockaddr *)&addr, addr_len, server_socket, &slave);
			} else {
				/* Wait .. we received something but nothing came in ? */
				perror("UDP receive error");
				printf("\nError on UDP packet reception (ret: %d)\n", len);
			}
		}

		/* Read user input if any */
        if (FD_ISSET(STDIN_FILENO, &tmp_read_fds)) {
            memset(buf, 0, BUF_SIZE);
            len = read(STDIN_FILENO, buf, BUF_SIZE);
			/* Do not know how to handle it yet, nothing defined. */
		}

		/* Handle module messages */
		if (FD_ISSET(slave.fd, &tmp_read_fds)) {
			int idx = 0;
			memset(buf, 0, BUF_SIZE);
			/* Get serial data and try to build a packet */
			len = read(slave.fd, buf, BUF_SIZE);
			if (len < 0) {
				perror("serial read error");
				/* FIXME : handle errors */
			} else if (len == 0) {
				/* Wait .. we received something but nothing came in ? */
				printf("\nError, got activity on serial link, but no data ...\n");
			}
			while (idx < len) {
				ret = dtplug_protocol_decode(buf[idx], &slave);
				/* Check return code to know if we have a valid packet */
				if (ret == -1) {
					/* Anything that's not part of a packet is printed as is (debug output) */
					printf("%c", buf[idx]);
				} else if (ret < 0) {
					printf("\nError in received packet. (ret: %d)\n", ret);
					/* FIXME : dump packet for debugging */
				} else if (ret == 0) {
					/* Packet is being built */
				} else {
					/* Valid packet received */
					handle_module_data(&slave);
				}
				idx++;
			}
		}

	} /* End of infinite loop */

	close(slave.fd);
	close(server_socket);
	return 0;
}


