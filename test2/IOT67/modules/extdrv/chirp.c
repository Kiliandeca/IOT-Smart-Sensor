/****************************************************************************
 *   extdrv/chirp.c
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


#include "core/system.h"
#include "lib/errno.h"
#include "drivers/i2c.h"
#include "extdrv/chirp.h"
#include "lib/utils.h"

/***************************************************************************** */
/*          Support for CHIRP101 temperature sensors from Texas Instrument       */
/***************************************************************************** */
/* This driver is made for the CHIRP101 version of the chip, though there's few
 * diferences between the CHIRP100 and CHIRP101 version.
 * This driver does not handle the SMBus Alert command.
 */


enum chirp_internal_reg_numbers {
	CHIRP_REG_CAP = 0,
	CHIRP_REG_LIGHT_REQ = 3,
	CHIRP_REG_LIGHT = 4,
	CHIRP_REG_TEMPERATURE = 5,
	CHIRP_REG_RESET = 6,
};



int chirp_probe_sensor(struct chirp_sensor_config* conf)
{
	char cmd_buf = (conf->addr | I2C_READ_BIT);

	/* Did we already probe the sensor ? */
	if (conf->probe_ok != 1) {
		conf->probe_ok = i2c_read(conf->bus_num, &cmd_buf, 1, NULL, NULL, 0);
	}
	return conf->probe_ok;
}



#define CMD_BUF_SIZE 3
int chirp_sensor_read(struct chirp_sensor_config* conf, uint8_t reg)
{
	int ret = 0;
	uint16_t val = 0;
	char cmd_buf[CMD_BUF_SIZE] = { conf->addr, 0, (conf->addr | I2C_READ_BIT), };
	char ctrl_buf[CMD_BUF_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };

	if (chirp_probe_sensor(conf) != 1) {
		return -ENODEV;
	}
	cmd_buf[1] = reg;

	ret = i2c_read(conf->bus_num, cmd_buf, CMD_BUF_SIZE, ctrl_buf, (char*)&val, 2);

	/* Swap to our endianness */
	val = (uint16_t)ntohs(val);

	if (ret != 2) {
		conf->probe_ok = 0;
		return ret;
	}

	return (int)val;
}

int chirp_sensor_write(struct chirp_sensor_config* conf, uint8_t val)
{
	int ret = 0;
	char cmd[CMD_BUF_SIZE] = { conf->addr, };

	if (chirp_probe_sensor(conf) != 1) {
		return -ENODEV;
	}

	cmd[1] = val;
	ret = i2c_write(conf->bus_num, cmd, 2, NULL);

	return ret;
}




/* Capacitance Read
 * Performs a non-blocking read of the capacitance from the sensor.
 * Return value(s):
 *   Upon successfull completion, returns the capacitance read (uint16_t).
 *   On error, returns a negative integer equivalent to errors from glibc.
 */
int chirp_sensor_cap_read(struct chirp_sensor_config* conf)
{
	return chirp_sensor_read(conf, CHIRP_REG_CAP);
}


/* Temp Read
 * Performs a non-blocking read of the temperature from the sensor.
 * Return value(s):
 *   Upon successfull completion, returns the temperature read (uint16_t).
 *   On error, returns a negative integer equivalent to errors from glibc.
 */
int chirp_sensor_temp_read(struct chirp_sensor_config* conf)
{
	return chirp_sensor_read(conf, CHIRP_REG_TEMPERATURE);
}


/* Start a conversion of the light sensor. */
int chirp_sensor_start_light_conversion(struct chirp_sensor_config* conf)
{
	return chirp_sensor_write(conf, CHIRP_REG_LIGHT_REQ);
}

/* Light Read
 * Performs a non-blocking read of the light from the sensor.
 * Return value(s):
 *   Upon successfull completion, returns the ambiant light read (uint16_t).
 *   On error, returns a negative integer equivalent to errors from glibc.
 */
int chirp_sensor_light_read(struct chirp_sensor_config* conf)
{
	return chirp_sensor_read(conf, CHIRP_REG_LIGHT);
}



/* Sensor reset
 * Return value:
 *   Upon successfull completion, returns 2. On error, returns a negative integer
 *   equivalent to errors from glibc.
 */
int chirp_reset(struct chirp_sensor_config* conf)
{
	return chirp_sensor_write(conf, CHIRP_REG_RESET);
}


