/****************************************************************************
 *   extdrv/dht11.h
 *
 *
 *
 * Copyright 2013 Nathael Pajani <nathael.pajani@ed3l.fr>
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
 *************************************************************************** */

#ifndef EXTDRV_DHT11_H
#define EXTDRV_DHT11_H

/*
 * Support for the DHT11 Temperature and Humidity sensor
 * Remember that the sensor operates on 3.5 to 5.5V, and that a level shifter must
 *   be used to interface with the DHT11 sensor.
 *
 * Note : This support uses a lot of "sleep()" calls, and is thus very ineficient
 *   and many data conversion can occur (checksum errors, though on all of my tests
 *   the errors were always on the lower bit, when the temp and humidity values
 *   were changing, and the scope always reported very stable signals, so I suspect
 *   that the error is not on the micro-controler side ... )
 * The addition of the "epsilon_error" with a value of 1 removed all errors on my
 *   side., while still getting coherent values.
 *
 * DHT11 protocol can be found here : http://embedded-lab.com/blog/?p=4333
 *
 */

#include "lib/stdint.h"

/***************************************************************************** */
/* DHT11 Humidity and temp sensor configuration : Set up the GPIO used for
 * communication with the DHT11 sensor
 */
void dht11_config(const struct pio* gpio);

/* Display both temperature and humidity values on the given serial line.
 * serial_num is the serial line number (0 or 1)
 * epsilon_error is the allowed error on the checksum
 */
void dht11_display(int serial_num, int epsilon_error);



#endif /* EXTDRV_DHT11_H */

