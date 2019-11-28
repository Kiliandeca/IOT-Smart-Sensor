/****************************************************************************
 *   extdrv/tmp101_temp_sensor.c
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


#include "core/system.h"
#include "lib/errno.h"
#include "drivers/i2c.h"
#include "extdrv/tmp101_temp_sensor.h"


/***************************************************************************** */
/*          Support for TMP101 temperature sensors from Texas Instrument       */
/***************************************************************************** */
/* This driver is made for the TMP101 version of the chip, though there's few
 * diferences between the TMP100 and TMP101 version.
 * This driver does not handle the SMBus Alert command.
 */


enum tmp10x_internal_reg_numbers {
	TMP_REG_TEMPERATURE = 0,
	TMP_REG_CONFIG,
	TMP_REG_ALERT_LOW,
	TMP_REG_ALERT_HIGH,
};


/* Aditional defines, not exported to userspace */
/* Mode config */
#define TMP_SHUTDOWN_MODE_ON            (1 << 0)
#define TMP_THERMOSTAT_COMPARATOR_MODE  (0 << 1)
#define TMP_THERMOSTAT_INTERRUPT_MODE   (1 << 1)
/* Alert signal polarity */
#define TMP_ALERT_POLARITY_LOW          (0 << 2)
#define TMP_ALERT_POLARITY_HIGH         (1 << 2)
/* One-shot measurement trigger */
#define TMP_ONE_SHOT_TRIGGER            (1 << 7)



/* Check the sensor presence, return 1 if sensor was found.
 * This is a basic check, it could be anything with the same address ...
 * addr : the sensor address on most significant bits.
 */
int tmp101_probe_sensor(struct tmp101_sensor_config* conf)
{
	char cmd_buf = (conf->addr | I2C_READ_BIT);

	/* Did we already probe the sensor ? */
	if (conf->probe_ok != 1) {
		conf->probe_ok = i2c_read(conf->bus_num, &cmd_buf, 1, NULL, NULL, 0);
	}
	return conf->probe_ok;
}

/* Convert raw temperature data (expressed as signed value of 16 times the
 * actual temperature in the twelve higher bits of a sixteen bits wide value)
 * into a decimal integer value of ten times the actual temperature.
 * The value returned is thus in tenth of degrees centigrade.
 */
int tmp101_convert_to_deci_degrees(uint16_t raw)
{
	return (((int16_t)raw * 10) >> 8);
}

/* Temp Read
 * Performs a non-blocking read of the temperature from the sensor.
 * 'raw' and 'deci_degrees' : integer addresses for conversion result, may be NULL.
 * Return value(s):
 *   Upon successfull completion, returns 0 and the temperature read is placed in the
 *   provided integers. On error, returns a negative integer equivalent to errors from glibc.
 */
#define CMD_BUF_SIZE 3
int tmp101_sensor_read(struct tmp101_sensor_config* conf, uint16_t* raw, int* deci_degrees)
{
	int ret = 0;
	uint16_t temp = 0;
	char cmd_buf[CMD_BUF_SIZE] = { conf->addr, TMP_REG_TEMPERATURE, (conf->addr | I2C_READ_BIT), };
	char ctrl_buf[CMD_BUF_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };

	if (tmp101_probe_sensor(conf) != 1) {
		return -ENODEV;
	}

	/* Read the requested data */
	if (conf->last_accessed_register == TMP_REG_TEMPERATURE) {
		/* No need to switch back to temperature register */
		ret = i2c_read(conf->bus_num, (cmd_buf + 2), 1, (ctrl_buf + 2), (char*)&temp, 2);
	} else {
		/* Send (write) temperature register address to TMP101 internal pointer */
		ret = i2c_read(conf->bus_num, cmd_buf, CMD_BUF_SIZE, ctrl_buf, (char*)&temp, 2);
		conf->last_accessed_register = TMP_REG_TEMPERATURE;
	}

	if (ret != 2) {
		conf->probe_ok = 0;
		return ret;
	}

	if (raw != NULL) {
		*raw = byte_swap_16(temp);
	}
	if (deci_degrees != NULL) {
		*deci_degrees = tmp101_convert_to_deci_degrees(byte_swap_16(temp));
	}
	return 0;
}


/* Sensor config
 * Performs default configuration of the temperature sensor.
 * The sensor is thus placed in shutdown mode, the thermostat is in interrupt mode,
 * and the polarity is set to active high.
 * Return value :
 *   Upon successfull completion, returns 0. On error, returns a negative integer
 *   equivalent to errors from glibc.
 */
#define CONF_BUF_SIZE 4
int tmp101_sensor_config(struct tmp101_sensor_config* conf)
{
	int ret = 0;
	char cmd[CONF_BUF_SIZE] = { conf->addr, TMP_REG_CONFIG, };

	if (tmp101_probe_sensor(conf) != 1) {
		return -ENODEV;
	}

	/* Store the new configuration */
	conf->actual_config = (TMP_SHUTDOWN_MODE_ON | TMP_THERMOSTAT_INTERRUPT_MODE | TMP_ALERT_POLARITY_HIGH);
	conf->actual_config |= (conf->resolution & (0x03 << 5));
	cmd[2] = conf->actual_config;
	ret = i2c_write(conf->bus_num, cmd, 3, NULL);
	conf->last_accessed_register = TMP_REG_CONFIG;
	if (ret != 3) {
		conf->probe_ok = 0;
		return ret;
	}
	return 0; /* Config success */
}

/* Start a conversion when the sensor is in shutdown mode. */
int tmp101_sensor_start_conversion(struct tmp101_sensor_config* conf)
{
	int ret = 0;
	char cmd[CONF_BUF_SIZE] = { conf->addr, TMP_REG_CONFIG, };

	if (tmp101_probe_sensor(conf) != 1) {
		return -ENODEV;
	}

	cmd[2] = conf->actual_config;
	cmd[2] |= TMP_ONE_SHOT_TRIGGER;
	ret = i2c_write(conf->bus_num, cmd, 3, NULL);
	conf->last_accessed_register = TMP_REG_CONFIG;
	if (ret != 3) {
		conf->probe_ok = 0;
		return ret;
	}
	return 0; /* Conversion start success */
}


