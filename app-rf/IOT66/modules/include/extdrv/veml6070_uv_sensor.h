/****************************************************************************
 *   extdrv/veml6070_uv_sensor.h
 *
 * VEML6070 I2C UV sensor driver
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
 *************************************************************************** */

#ifndef EXTDRV_VEML6070_H
#define EXTDRV_VEML6070_H

#include "lib/stdint.h"



/* VEML6070 sensor instance data.
 * Note that the veml6070 sensor adress cannot be changed.
 */
struct veml6070_sensor_config {
    uint8_t addr;
	uint8_t bus_num;
    uint8_t probe_ok;
    uint8_t actual_config;
};


/* Defines for command byte */
#define VEML6070_ACK               (1 << 5)
#define VEML6070_ACK_THD_102       (0)
#define VEML6070_ACK_THD_145       (1 << 4)
#define VEML6070_NO_ACK            (0)
#define VEML6070_INTEG_05T         (0x00 << 2)
#define VEML6070_INTEG_1T          (0x01 << 2)
#define VEML6070_INTEG_2T          (0x02 << 2)
#define VEML6070_INTEG_4T          (0x03 << 2)
#define VEML6070_DISABLE           (1 << 0)
#define VEML6070_ENABLE            (0)


/* Check the sensor presence, return 1 if found */
int veml6070_probe_sensor(struct veml6070_sensor_config* conf);


/* UV Read
 * Performs a read of the uv data from the sensor.
 * 'uv_raw': integer addresses for conversion result.
 * Return value(s):
 *   Upon successfull completion, returns 0 and the value read is placed in the provided integer.
 *   On error, returns a negative integer equivalent to errors from glibc.
 */
int veml6070_sensor_read(struct veml6070_sensor_config* conf, uint16_t* uv_raw);


/* Sensor config
 * Performs default configuration of the UV sensor.
 * Return value:
 *   Upon successfull completion, returns 0. On error, returns a negative integer
 *   equivalent to errors from glibc.
 */
int veml6070_configure(struct veml6070_sensor_config* conf);


#endif /* EXTDRV_VEML6070_H */

