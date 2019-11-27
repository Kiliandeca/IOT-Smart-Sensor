/****************************************************************************
 *  LightHouse UDP to RF Sub1GHz module bridge.
 *
 *   handlers.h
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

#ifndef HANDLERS_H
#define HANDLERS_H

#include <stdint.h>
#include <sys/types.h>
#include "dtplug_protocol_host.h"



int handle_module_data(struct line_transceiver* slave);


int handle_udp_request(char* buf, int len, struct sockaddr* addr, socklen_t addr_len,
						int sock, struct line_transceiver* slave);


#endif /* HANDLERS_H */
