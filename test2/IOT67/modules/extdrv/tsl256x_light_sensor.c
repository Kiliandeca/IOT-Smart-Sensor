/****************************************************************************
 *   extdrv/tsl256x_light_sensor.c
 *
 * TSL256x I2C luminosity and IR sensor driver
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


#include "lib/stdint.h"
#include "core/system.h"
#include "lib/errno.h"
#include "drivers/i2c.h"

#include "extdrv/tsl256x_light_sensor.h"



/* Check the sensor presence, return 1 if found
 * This is done by writing to the control register to set the power state to ON and
 *   reading the register to check the value, as stated in the TSL256x documentation page 14,
 *   (Register Set definitions)
 * FIXME : Never managed to read the required value, though the sensor is running and
 *     provides seemingly accurate values.
 */
#define PROBE_BUF_SIZE  3
int tsl256x_probe_sensor(struct tsl256x_sensor_config* conf)
{
	int ret = 0;
	char cmd_buf[PROBE_BUF_SIZE] = { conf->addr, TSL256x_CMD(control), TSL256x_POWER_ON, };
	char ctrl_buf[PROBE_BUF_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };
	uint8_t control_val = 0;

	/* Did we already probe the sensor ? */
	if (conf->probe_ok != 1) {
		ret = i2c_write(conf->bus_num, cmd_buf, PROBE_BUF_SIZE, NULL);
		if (ret != PROBE_BUF_SIZE) {
			return 0;
		}
		msleep(500);
		cmd_buf[2] = (conf->addr | I2C_READ_BIT);
		ret = i2c_read(conf->bus_num, cmd_buf, PROBE_BUF_SIZE, ctrl_buf, &control_val, 1);
		/* FIXME : check that control_val is TSL256x_POWER_ON ... */
		if (ret == 1) {
			conf->probe_ok = 1;
		}
	}
	return conf->probe_ok;
}



/* FIXME: add comments, and make it work right ... never managed to read the ID given in
 *   the documentation
 */

#define ID_BUF_SIZE  3
int tsl256x_read_id(struct tsl256x_sensor_config* conf)
{
	int ret = 0;
	char cmd_buf[ID_BUF_SIZE] = { conf->addr, TSL256x_CMD(part_id), (conf->addr | I2C_READ_BIT), };
	char ctrl_buf[ID_BUF_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };
	uint8_t id = 0;

	/* Did we already probe the sensor ? */
	if (conf->probe_ok != 1) {
		return 0;
	}
	ret = i2c_read(conf->bus_num, cmd_buf, ID_BUF_SIZE, ctrl_buf, &id, 1);
	if (ret != 1) {
		conf->probe_ok = 0;
		return ret;
	}
	return id;
}


/* Lux Read
 * Performs a non-blocking read of the luminosity from the sensor.
 * 'lux' 'ir' and 'comb': integer addresses for conversion result, may be NULL.
 * Return value(s):
 *   Upon successfull completion, returns 0 and the luminosity read is placed in the
 *   provided integer(s). On error, returns a negative integer equivalent to errors from
 *   glibc.
 */
#define READ_BUF_SIZE  3
int tsl256x_sensor_read(struct tsl256x_sensor_config* conf, uint16_t* comb, uint16_t* ir, uint32_t* lux)
{
	int ret = 0;
	char cmd_buf[READ_BUF_SIZE] = { conf->addr, TSL256x_CMD(data), (conf->addr | I2C_READ_BIT), };
	char ctrl_buf[READ_BUF_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };
	uint8_t data[4];
	uint16_t comb_raw = 0, ir_raw = 0;

	ret = i2c_read(conf->bus_num, cmd_buf, READ_BUF_SIZE, ctrl_buf, data, 4);
	if (ret != 4) {
		conf->probe_ok = 0;
		return ret;
	}
	comb_raw = (data[0] & 0xFF) | ((data[1] << 8) & 0xFF00);
	ir_raw = (data[2] & 0xFF) | ((data[3] << 8) & 0xFF00);

	if (comb != NULL) {
		*comb = comb_raw;
	}
	if (ir != NULL) {
		*ir = ir_raw;
	}
	if (lux != NULL) {
		*lux = calculate_lux(conf, comb_raw, ir_raw);
	}

	return 0;
}


/* Sensor config
 * Performs default configuration of the luminosity sensor.
 * FIXME : Add more comments about the behavior and the resulting configuration.
 * Return value:
 *   Upon successfull completion, returns 0. On error, returns a negative integer
 *   equivalent to errors from glibc.
 */
#define CONF_BUF_SIZE 3
int tsl256x_configure(struct tsl256x_sensor_config* conf)
{
	int ret = 0;
	char cmd_buf[CONF_BUF_SIZE] = { conf->addr, TSL256x_CMD(timing), 0, };

	cmd_buf[2] = (conf->gain | conf->integration_time);

	if (tsl256x_probe_sensor(conf) != 1) {
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



/***************************************************************************** */
/*
 * lux equation approximation without floating point calculations
 *
 * Description:
 *   Calculate the approximate illuminance (lux) given the raw channel values of
 *   the TSL2560. The equation if implemented as a piece−wise linear approximation.
 *
 * Arguments:
 * uint16_t ch0 − raw channel value from channel 0 of TSL2560
 * uint16_t ch1 − raw channel value from channel 1 of TSL2560
 *
 * Return: uint32_t − the approximate illuminance (lux)
 *
 */
uint32_t calculate_lux(struct tsl256x_sensor_config* conf, uint16_t ch0, uint16_t ch1)
{
	/* First, scale the channel values depending on the gain and integration time
	 * 16X, 402mS is nominal.
	 * Scale if integration time is NOT 402 msec */
	uint32_t chScale = 0;
	uint32_t channel1 = 0, channel0 = 0;
	uint32_t ratio = 0, lux = 0;
	uint32_t b = 0, m = 0;

	switch (conf->integration_time) {
		 case TSL256x_INTEGRATION_13ms: /* 13.7 msec */
			chScale = CHSCALE_TINT0;
			break;
		 case TSL256x_INTEGRATION_100ms: /* 101 msec */
			chScale = CHSCALE_TINT1;
			break;
		case TSL256x_INTEGRATION_400ms: /* 402 msec */
		default: /* assume no scaling */
			chScale = (1 << CH_SCALE);
			break;
	}

	/* Scale if gain is NOT 16X */
	if (conf->gain == TSL256x_LOW_GAIN) {
		chScale = chScale << 4; /* Scale 1X to 16X */
	}

	// Scale the channel values */
	channel0 = (ch0 * chScale) >> CH_SCALE;
	channel1 = (ch1 * chScale) >> CH_SCALE;

	/* Find the ratio of the channel values (Channel1/Channel0) */
	/* Protect against divide by zero */
	if (channel0 != 0) {
		ratio = (channel1 << (RATIO_SCALE + 1)) / channel0;
	}
	/* Round the ratio value */
	ratio = (ratio + 1) >> 1;

	/* Is ratio <= eachBreak ? */
	switch (conf->package) {
		case TSL256x_PACKAGE_T:
		case TSL256x_PACKAGE_FN:
		case TSL256x_PACKAGE_CL:
			if ((ratio >= 0) && (ratio <= K1T)) {
				b = B1T; m = M1T;
			} else if (ratio <=  K2T) {
				b = B2T; m = M2T;
			} else if (ratio <=  K3T) {
				b = B3T; m = M3T;
			} else if (ratio <=  K4T) {
				b = B4T; m = M4T;
			} else if (ratio <=  K5T) {
				b = B5T; m = M5T;
			} else if (ratio <=  K6T) {
				b = B6T; m = M6T;
			} else if (ratio <=  K7T) {
				b = B7T; m = M7T;
			} else if (ratio > K8T) {
				b = B8T; m = M8T;
			} break;
		case TSL256x_PACKAGE_CS:    /* CS package */
			if ((ratio >=  0) && (ratio <=  K1C)) {
				b = B1C; m = M1C;
			} else if (ratio <=  K2C) {
				b = B2C; m = M2C;
			} else if (ratio <=  K3C) {
				b = B3C; m = M3C;
			} else if (ratio <=  K4C) {
				b = B4C; m = M4C;
			} else if (ratio <=  K5C) {
				b = B5C; m = M5C;
			} else if (ratio <=  K6C) {
				b = B6C; m = M6C;
			} else if (ratio <=  K7C) {
				b = B7C; m = M7C;
			}else if (ratio > K8C) {
				b = B8C; m = M8C;
			} break;
	}
	lux = ((channel0 * b) - (channel1 * m));

	/* Do not allow negative lux value */
	if (lux < 0) {
		lux = 0;
	}
	/* Round lsb (2^(LUX_SCALE−1)) */
	lux += (1 << (LUX_SCALE - 1));
	/* Strip off fractional portion */
	lux = lux >> LUX_SCALE;

	return lux;
}

