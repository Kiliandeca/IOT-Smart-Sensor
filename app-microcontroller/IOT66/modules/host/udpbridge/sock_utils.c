/*****************************************************************************
 *
 * socket utils
 *
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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
 *****************************************************************************/

#include <stdio.h>  /* perror */
#include <string.h> /* memset */
#include <unistd.h> /* close, fcntl */
#include <fcntl.h>  /* F_SETFL and O_NONBLOCK for fcntl */

/* For socket stuff */
#include <sys/types.h> /* For portability for socket stuff */
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h> /* For TCP_NODELAY to disable nagle algorithm */


/* This is used to allow quicker sending of small packets on wires */
static void sox_disable_nagle_algorithm(int socket)
{
	const int on = 1;
	setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, (char*)&on, sizeof( int ));
}

/* Creates a TCP socket on the specified "port", bind to port and setup to
 * accept connections with listen, with up to "nb_pending" pending connections
 * in the queue.
 * Returns the socket, after disabling nagle algorithm.
 * If "our" is not null, it will be used to store the listening socket information.
 * If "port" is 0, then "our" must contain all requiered information for bind call.
 */
int socket_tcp_server(int port, int nb_pending, struct sockaddr_in* our)
{
	struct sockaddr_in local;
	struct sockaddr_in* tmp;
	int s;
	int optval = 1;

	if (our == NULL ) {
		tmp = &local;
		if (port == 0) {
			fprintf(stderr, "No port and no address information provided, won't be able to bind, aborting\n");
			return -1;
		}
	} else {
		tmp = our;
	}

	if (port != 0) {
		memset(tmp, 0, sizeof(struct sockaddr_in));
		tmp->sin_family = AF_INET;
		tmp->sin_addr.s_addr = htonl(INADDR_ANY);
		tmp->sin_port = htons((short)port);
	}

	if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("Unable to create TCP socket");
		return -1;
	}

	if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (void *)&optval, sizeof(int)) == -1) {
		perror("Unable to set reuseaddress on socket");
		goto err_close;
	}
	if (setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, (void *)&optval, sizeof(int)) == -1) {
		perror("Unable to set keepalive on socket");
		goto err_close;
	}

	if (bind(s, (struct sockaddr *)tmp, sizeof(struct sockaddr_in)) < 0) {
		perror("Unable to bind TCP port");
		goto err_close;
	}

	if (listen(s, nb_pending)) {
		perror("Unable to listen to TCP port");
		goto err_close;
	}

	sox_disable_nagle_algorithm(s);
	return s;

err_close:
	close(s);
	return -1;
}


/* Creates an UDP socket on the specified "port" and bind to port but no specific interface.
 * Returns the socket.
 * If "our" is not null, it will be used to store the listening socket information.
 * "port" must not be 0..
 */
int create_bound_udp_socket(int port, struct sockaddr_in* our)
{
	struct sockaddr_in local;
	struct sockaddr_in* tmp;
	int s;
	int optval = 1;

	if (port == 0) {
		return -1;
	}

	if (our == NULL ) {
		tmp = &local;
	} else {
		tmp = our;
	}

	memset(tmp, 0, sizeof(struct sockaddr_in));
	tmp->sin_family = AF_INET;
	tmp->sin_addr.s_addr = htonl(INADDR_ANY);
	tmp->sin_port = htons((short)port);

	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("Unable to create UDP socket");
		return -1;
	}

	if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (void *)&optval, sizeof(int)) == -1) {
		perror("Unable to set reuseaddress on socket");
		goto err_close;
	}

	if (bind(s, (struct sockaddr *)tmp, sizeof(struct sockaddr_in)) < 0) {
		perror("Unable to bind UDP port");
		goto err_close;
	}

	return s;

err_close:
	close(s);
	return -1;
}



/* Creates an UDP socket bound to no specific interface.
 * The kernel choses the port to bind to.
 * Returns the socket.
 */
int create_broadcast_udp_socket(void)
{
	struct sockaddr_in local;
	struct sockaddr_in* tmp = &local;
	int s;
	int optval = 1;

	memset(tmp, 0, sizeof(struct sockaddr_in));
	tmp->sin_family = AF_INET;
	tmp->sin_addr.s_addr = htonl(INADDR_ANY);
	tmp->sin_port = 0;

	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("Unable to create UDP socket");
		return -1;
	}

	if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (void *)&optval, sizeof(int)) == -1) {
		perror("Unable to set reuseaddress on socket");
		goto err_close;
	}

	if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, (void *)&optval, sizeof(int)) == -1) {
		perror("Unable to set broadcast on socket");
		goto err_close;
	}

	if (bind(s, (struct sockaddr *)tmp, sizeof(struct sockaddr_in)) < 0) {
		perror("Unable to bind UDP port");
		goto err_close;
	}

	return s;

err_close:
	close(s);
	return -1;
}


