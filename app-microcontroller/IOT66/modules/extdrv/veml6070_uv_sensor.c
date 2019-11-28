/****************************************************************************
 *   extdrv/veml6070_uv_sensor.c
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


#include "core/system.h"
#include "lib/errno.h"
#include "drivers/i2c.h"
#include "extdrv/veml6070_uv_sensor.h"



/* Check the sensor presence, return 1 if found */
int veml6070_probe_sensor(struct veml6070_sensor_config* conf)
{
    char cmd_buf = (conf->addr | I2C_READ_BIT);
	uint8_t dropped;

    /* Did we already probe the sensor ? */
    if (conf->probe_ok != 1) {
        conf->probe_ok = i2c_read(conf->bus_num, &cmd_buf, 1, NULL, &dropped, 1);
    }
    return conf->probe_ok;
}

/* UV Read
 * Performs a read of the uv data from the sensor.
 * 'uv_raw': integer addresses for conversion result.
 * Return value(s):
 *   Upon successfull completion, returns 0 and the luminosity read is placed in the
 *   provided integer(s). On error, returns a negative integer equivalent to errors from
 *   glibc.
 */
int veml6070_sensor_read(struct veml6070_sensor_config* conf, uint16_t* uv_raw)
{
    int ret = 0;
    char cmd_buf = 0;
    uint8_t data = 0;

    if (conf->probe_ok != 1) {
	    if (veml6070_probe_sensor(conf) != 1) {
			return -ENODEV;
	    }
		msleep(1);
	}
    if (uv_raw == NULL) {
        return -EINVAL;
    }

	/* Start by reading MSB */
	cmd_buf = ((conf->addr + 2) | I2C_READ_BIT);
    ret = i2c_read(conf->bus_num, &cmd_buf, 1, NULL, &data, 1);
    if (ret != 1) {
		conf->probe_ok = 0;
        return ret;
    }
    *uv_raw = ((uint16_t)data << 8) & 0xFF00;
    msleep(1);
	/* And read LSB */
    cmd_buf = (conf->addr | I2C_READ_BIT);
    ret = i2c_read(conf->bus_num, &cmd_buf, 1, NULL, &data, 1);
    if (ret != 1) {
		conf->probe_ok = 0;
        return ret;
    }
    *uv_raw |= (uint16_t)data & 0x00FF;

    return 0;
}


/* Sensor config
 * Performs default configuration of the UV sensor.
 * Return value:
 *   Upon successfull completion, returns 0. On error, returns a negative integer
 *   equivalent to errors from glibc.
 */
#define CONF_BUF_SIZE 2
int veml6070_configure(struct veml6070_sensor_config* conf)
{
    int ret = 0;
    char cmd_buf[CONF_BUF_SIZE] = { conf->addr, (VEML6070_NO_ACK | VEML6070_INTEG_1T | VEML6070_ENABLE), };

    if (veml6070_probe_sensor(conf) != 1) {
        return -ENODEV;
    }
	msleep(1);
    ret = i2c_write(conf->bus_num, cmd_buf, CONF_BUF_SIZE, NULL);
    if (ret != CONF_BUF_SIZE) {
		conf->probe_ok = 0;
        return -EIO;
    }
    return 0;
}




