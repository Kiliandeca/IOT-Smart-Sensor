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

#ifndef SOCK_UTILS_H
#define SOCK_UTILS_H

#include <netinet/in.h> /* struct sockaddr_in */

/* Creates a socket on the specified "port", bind to port and setup to
   accept connections with listen, with up to "nb_pending" pending connections
   in the queue.
   Returns the socket, after disabling nagle algorithm.
   If "our" is not null, it will be used to store the listening socket information.
 */
int socket_tcp_server(int port, int nb_pending, struct sockaddr_in* our);


/* Creates an UDP socket on the specified "port" and bind to port but no specific interface.
 * Returns the socket.
 * If "our" is not null, it will be used to store the listening socket information.
 * "port" must not be 0..
 */
int create_bound_udp_socket(int port, struct sockaddr_in* our);


/* Creates an UDP socket bound to no specific interface.
 * The kernel choses the port to bind to.
 * Returns the socket.
 */
int create_broadcast_udp_socket(void);

#endif /* SOCK_UTILS_H */

