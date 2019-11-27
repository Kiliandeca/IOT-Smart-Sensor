/*********************************************************************
 *
 *   Serial utility functions
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
 *********************************************************************/
#ifndef SERIAL_UTILS_H
#define SERIAL_UTILS_H

/* Setup serial comunication, using name if given or saved name if name is NULL
 *  SERIAL_BAUD  B38400
 *  c_cflag  (CS7 | PARENB | CREAD | CLOCAL) (7e1)
 */
int serial_setup(char* name);

#endif /* SERIAL_UTILS_H */

