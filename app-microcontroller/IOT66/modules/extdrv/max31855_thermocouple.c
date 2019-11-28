/****************************************************************************
 *   extdrv/max31855_thermocouple.c
 *
 *
 * Copyright 2015 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#include "core/system.h"
#include "core/pio.h"
#include "lib/errno.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"
#include "extdrv/max31855_thermocouple.h"


/* Support for thermocouple temperature sensors using Maxim's MAX31855 themocouple
 *   to digital converter.
 */


/* Convert raw temperature data (expressed as ....)
 * into a centimal integer value of ten times the actual temperature.
 * The value returned is thus in tenth of celcius degrees.
 */
int32_t max31855_convert_to_centi_degrees(uint32_t raw)
{
	int32_t deg = ((int32_t) raw) >> 18;
	return ((deg * 100) >> 2);
}

/* Temp Read
 * Performs a non-blocking read of the temperature from the sensor.
 * 'raw' and 'centi_degrees' : integer addresses for conversion result, may be NULL.
 * Return value(s):
 *   Upon successfull completion, returns 0 and the temperature read is placed in the integers pointed to
 *     by 'raw' and 'centi_degrees' pointers.
 *   Return -EBUSY when unable to get SPI bus mutex or -ENODEV if the Fault bit is set in the temperature
 *     word. If -ENODEV is returned and 'raw' is not NULL then raw will be updated with the three bits
 *     indicating the error condition instead of the temparature.
 */
int max31855_sensor_read(const struct max31855_sensor_config* conf, uint16_t* raw, int* centi_degrees)
{
	int temp = 0;
	uint32_t data = 0;

	/* Get hold of the SPI bus */
	if (spi_get_mutex(conf->ssp_bus_num) != 1) {
		return -EBUSY;
	}
	/* Read data */
	gpio_clear(conf->chip_select);
	spi_transfer_multiple_frames(conf->ssp_bus_num, NULL, (uint8_t*)(&data), 4, 8);
	gpio_set(conf->chip_select);
	/* Release SPI bus */
	spi_release_mutex(conf->ssp_bus_num);

	/* Swap bytes ! */
	data = byte_swap_32(data);

	/* Check for fault info */
	if (data & MAX31855_TH_FAULT) {
		if (raw != NULL) {
			*raw = (data & MAX31855_TH_ERROR_BITS_MASK);
		}
		return -ENODEV;
	}

	/* Convert result */
	temp = max31855_convert_to_centi_degrees(data);
	if (raw != NULL) {
		*raw = data;
	}
	if (centi_degrees != NULL) {
		*centi_degrees = temp;
	}

	return 0;
}


/* Sensor config */
void max31855_sensor_config(const struct max31855_sensor_config* conf)
{
	/* Configure slave select pin as GPIO and set initial level to 1 (not selected) */
	config_gpio(&(conf->chip_select), LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 1);
}


