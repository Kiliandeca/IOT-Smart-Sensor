/****************************************************************************
 *   extdrv/chirp.h
 *
 *
 * Copyright 2017 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef EXTDRV_CHIRP_H
#define EXTDRV_CHIRP_H

#include "lib/stdint.h"


/***************************************************************************** */
/*          Support for chirp sensor from miceuz (twitter.com/miceuz)          */
/*                https://wemakethings.net/chirp/                              */
/***************************************************************************** */


struct chirp_sensor_config {
	uint8_t addr;
	uint8_t bus_num;
	uint8_t probe_ok;
};


/* Check the sensor presence, return 1 if found
 * This is a basic check, it could be anything with the same address ...
 */
int chirp_probe_sensor(struct chirp_sensor_config* conf);


/* Capacitance Read
 * Performs a non-blocking read of the capacitance from the sensor.
 * Return value(s):
 *   Upon successfull completion, returns the capacitance read (uint16_t).
 *   On error, returns a negative integer equivalent to errors from glibc.
 */
int chirp_sensor_cap_read(struct chirp_sensor_config* conf);

/* Temp Read
 * Performs a non-blocking read of the temperature from the sensor.
 * Return value(s):
 *   Upon successfull completion, returns the temperature read (uint16_t).
 *   On error, returns a negative integer equivalent to errors from glibc.
 */
int chirp_sensor_temp_read(struct chirp_sensor_config* conf);


/* Start a conversion of the light sensor. */
int chirp_sensor_start_light_conversion(struct chirp_sensor_config* conf);

/* Light Read
 * Performs a non-blocking read of the light from the sensor.
 * Return value(s):
 *   Upon successfull completion, returns the ambiant light read (uint16_t).
 *   On error, returns a negative integer equivalent to errors from glibc.
 */
int chirp_sensor_light_read(struct chirp_sensor_config* conf);



/* Sensor reset
 * Return value:
 *   Upon successfull completion, returns 2. On error, returns a negative integer
 *   equivalent to errors from glibc.
 */
int chirp_reset(struct chirp_sensor_config* conf);


#endif /* EXTDRV_TEMP_H */

