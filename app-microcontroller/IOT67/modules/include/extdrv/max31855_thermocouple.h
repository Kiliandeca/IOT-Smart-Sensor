/****************************************************************************
 *   extdrv/max31855_thermocouple.h
 *
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef EXTDRV_MAX31855_H
#define EXTDRV_MAX31855_H

#include "lib/stdint.h"
#include "core/pio.h"

/* Support for thermocouple temperature sensors using Maxim's MAX31855 themocouple
 *   to digital converter.
 */

struct max31855_sensor_config {
	uint8_t ssp_bus_num;
	struct pio chip_select;
};

#define MAX31855_TH_TEMP_DATA(x)     (((x) >> 18) & 0x3FFF)
#define MAX31855_TH_REF_DATA(x)      (((x) >> 4) & 0x0FFF)
#define MAX31855_TH_FAULT            (0x01 << 16)
#define MAX31855_TH_ERROR_BITS_MASK  (0x07)
#define MAX31855_TH_ERR_SHORT_TO_VCC (0x01 << 2)
#define MAX31855_TH_ERR_SHORT_TO_GND (0x01 << 1)
#define MAX31855_TH_ERR_OPEN_CIRCUIT (0x01 << 0)

/* Convert raw temperature data (expressed as ....)
 * into a decimal integer value of ten times the actual temperature.
 * The value returned is thus in tenth of celcius degrees.
 */
int32_t max31855_convert_to_deci_degrees(uint32_t raw);

/* Temp Read
 * Performs a non-blocking read of the temperature from the sensor.
 * 'raw' and 'deci_degrees' : integer addresses for conversion result, may be NULL.
 * Return value(s):
 *   Upon successfull completion, returns 0 and the temperature read is placed in the
 *   integers pointed to by 'raw' and 'deci_degrees' pointers.
 */
int max31855_sensor_read(const struct max31855_sensor_config* conf, uint16_t* raw, int* deci_degrees);

/* Sensor config */
void max31855_sensor_config(const struct max31855_sensor_config* conf);


#endif /* EXTDRV_MAX31855_H */

