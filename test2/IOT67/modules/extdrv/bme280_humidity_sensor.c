/****************************************************************************
 *   extdrv/bme280_humidity_sensor.c
 *
 * BME280 I2C Barometric, humidity and temperature sensor driver
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

#include "extdrv/bme280_humidity_sensor.h"



/* Check the sensor presence, return 1 if found */
#define PROBE_BUF_SIZE  3
int bme280_probe_sensor(struct bme280_sensor_config* conf)
{
    char cmd_buf[PROBE_BUF_SIZE] = { conf->addr, BME280_REGS(chip_id), (conf->addr | I2C_READ_BIT), };
	char ctrl_buf[PROBE_BUF_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };
	uint8_t id = 0;

    /* Did we already probe the sensor ? */
    if (conf->probe_ok != 1) {
        conf->probe_ok = i2c_read(conf->bus_num, &cmd_buf, PROBE_BUF_SIZE, ctrl_buf, &id, 1);
		if ((conf->probe_ok == 1) && (id != BME280_ID)) {
			conf->probe_ok = 0;
		}
    }
    return conf->probe_ok;
}


/* Get calibration data from internal sensor memory
 * These values are required to compute the pressure, temperature and humidity values
 *   from the uncompensated "raw" values read from the sensor ADC result registers.
 * Calibration data for pressure and temperature is aligned, packed, and in little
 *    endian byte order, so it is stored directly to the conf->cal structure.
 * Calibration data for humidity is split among different registers and not aligned,
 *    so we use a temporary data buffer.
 * Note : On the LPC822 the I2C bus requires some time to go idle, so we need to
 *    sleep some time between two conscutive I2C access.
 *    this should be done in the I2C driver, but is not yet implemented, so it is
 *    done here. These msleep() calls may be removed for other micro-controllers, and
 *    when the I2C driver for the LPC822 is imporved.
 * Return value:
 *   Upon successfull completion, returns 0. On error, returns a negative integer
 *   equivalent to errors from glibc.
 */
#define CAL_CMD_SIZE  3
int bme280_get_calibration_data(struct bme280_sensor_config* conf)
{
	int ret = 0;
	char cmd_buf[CAL_CMD_SIZE] = { conf->addr, BME280_CAL_REGS(T), (conf->addr | I2C_READ_BIT), };
	char ctrl_buf[CAL_CMD_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };
	uint8_t data[BME280_CAL_REGS_H_LEN];

	if (bme280_probe_sensor(conf) != 1) {
		return -ENODEV;
	}

	/* Give some time for I2C bus to go idle */
	msleep(1);
	/* Easy part : Temperature and Presure calibration data is packed, aligned, and
	 *   in little endian byte order */
	/* Start by reading temperature calibration data */
	ret = i2c_read(conf->bus_num, cmd_buf, CAL_CMD_SIZE, ctrl_buf, &(conf->cal.T1), BME280_CAL_REGS_T_LEN);
	if (ret != BME280_CAL_REGS_T_LEN) {
		conf->probe_ok = 0;
		return ret;
	}
	msleep(1); /* Again some time for I2C bus to go idle */
	/* Read pressure calibration data */
	cmd_buf[1] = BME280_CAL_REGS(P);
	ret = i2c_read(conf->bus_num, cmd_buf, CAL_CMD_SIZE, ctrl_buf, &(conf->cal.P1), BME280_CAL_REGS_P_LEN);
	if (ret != BME280_CAL_REGS_P_LEN) {
		conf->probe_ok = 0;
		return ret;
	}
	msleep(1); /* ... */

	/* Read humidity calibration data. This one is split among bytes and not aligned. use
	 *   temporary data buffer and then copy to conf structure. */
	/* First part */
	cmd_buf[1] = BME280_CAL_REGS(Ha);
	ret = i2c_read(conf->bus_num, cmd_buf, CAL_CMD_SIZE, ctrl_buf, data, BME280_CAL_REGS_Ha_LEN);
	if (ret != BME280_CAL_REGS_Ha_LEN) {
		conf->probe_ok = 0;
		return ret;
	}
	msleep(1); /* ... */
	/* Second part */
	cmd_buf[1] = BME280_CAL_REGS(Hb);
	ret = i2c_read(conf->bus_num, cmd_buf, CAL_CMD_SIZE, ctrl_buf, (data + BME280_CAL_REGS_Ha_LEN), BME280_CAL_REGS_Hb_LEN);
	if (ret != BME280_CAL_REGS_Hb_LEN) {
		conf->probe_ok = 0;
		return ret;
	}
	/* And store in calibration structure */
	conf->cal.H1 = data[0];
	conf->cal.H2 = ((data[1] & 0xFF) | ((data[2] & 0xFF) << 8));
	conf->cal.H3 = data[3];
	conf->cal.H4 = (((data[4] & 0xFF) << 4) | (data[5] & 0x0F));
	conf->cal.H5 = (((data[6] & 0xFF) << 4) | ((data[5] & 0xF0) >> 4));
	conf->cal.H6 = data[7];

	return 0;
}

/* Sensor config
 * Performs configuration of the sensor and recovers calibration data from sensor's internal
 *   memory.
 * Return value:
 *   Upon successfull completion, returns 0. On error, returns a negative integer
 *   equivalent to errors from glibc.
 */
#define CONF_BUF_SIZE 7
int bme280_configure(struct bme280_sensor_config* conf)
{
    int ret = 0;
    char cmd_buf[CONF_BUF_SIZE] = {
			conf->addr,
			BME280_REGS(ctrl_humidity), BME280_CTRL_HUM(conf->humidity_oversampling),
			BME280_REGS(ctrl_measure),
				BME280_CTRL_MEA(conf->pressure_oversampling, conf->temp_oversampling, conf->mode),
			BME280_REGS(config), BME280_CONFIG(conf->standby_len, conf->filter_coeff),
		 };

    if (bme280_probe_sensor(conf) != 1) {
        return -ENODEV;
    }
	/* Send the configuration */
    ret = i2c_write(conf->bus_num, cmd_buf, CONF_BUF_SIZE, NULL);
    if (ret != CONF_BUF_SIZE) {
		conf->probe_ok = 0;
        return -EIO;
    }
	/* Get the calibration data */
	ret = bme280_get_calibration_data(conf);

    return ret;
}


/* Humidity, Temperature and Pressure Read
 * Performs a read of the data from the sensor.
 * 'hum', 'temp' and 'pressure': integer addresses for conversion result.
 * Return value(s):
 *   Upon successfull completion, returns 0 and the raw sensor values read are placed in the
 *   provided integer(s). On error, returns a negative integer equivalent to errors from
 *   glibc.
 */
#define READ_CMD_SIZE   3
int bme280_sensor_read(struct bme280_sensor_config* conf, uint32_t* pressure, uint32_t* temp, uint16_t* hum)
{
    int ret = 0;
    char cmd_buf[READ_CMD_SIZE] = { conf->addr, BME280_REGS(raw_data), (conf->addr | I2C_READ_BIT), };
	char ctrl_buf[READ_CMD_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };
    uint8_t data[BME280_DATA_SIZE];

    if (conf->probe_ok != 1) {
	    if (bme280_probe_sensor(conf) != 1) {
			return -ENODEV;
	    }
		msleep(1);
	}

	/* Start by reading all data */
    ret = i2c_read(conf->bus_num, cmd_buf, READ_CMD_SIZE, ctrl_buf, data, BME280_DATA_SIZE);
    if (ret != BME280_DATA_SIZE) {
		conf->probe_ok = 0;
        return ret;
    }
	if (pressure != NULL) {
		*pressure = BME280_DATA_20(data[0], data[1], data[2]);
	}
	if (temp != NULL) {
		*temp = BME280_DATA_20(data[3], data[4], data[5]);
	}
	if (hum != NULL) {
		*hum = BME280_DATA_16(data[6], data[7]);
	}

    return 0;
}



/* Compute actual temperature from uncompensated temperature
 * Param :
 *  - conf : bme280_sensor_configuration structure, with calibration data read from sensor
 *  - utemp : uncompensated (raw) temperature value read from sensor
 * Returns the value in 0.01 degree Centigrade
 * Output value of "5123" equals 51.23 DegC.
 */
int bme280_compensate_temperature(struct bme280_sensor_config* conf, int utemp)
{
	int tmp1 = 0, tmp2 = 0;
	int temperature = 0;

	/* Calculate tmp1 */
	tmp1 = ((((utemp >> 3) - ((int)conf->cal.T1 << 1))) * conf->cal.T2) >> 11;
	/* Calculate tmp2 */
	tmp2 = (((utemp >> 4) - (int)conf->cal.T1) * ((utemp >> 4) - (int)conf->cal.T1)) >> 12;
	tmp2 = (tmp2 * conf->cal.T3) >> 14;
	/* Calculate t_fine */
	conf->fine_temp = tmp1 + tmp2;
	/* Calculate temperature */
	temperature = (conf->fine_temp * 5 + 128) >> 8;
	return temperature;
}

/* Compute actual pressure from uncompensated pressure
 * Returns the value in Pascal(Pa) or 0 on error (invalid value which would cause division by 0).
 * Output value of "96386" equals 96386 Pa = 963.86 hPa = 963.86 millibar
 */
uint32_t bme280_compensate_pressure(struct bme280_sensor_config* conf, int uncomp_pressure)
{
	int tmp1 = 0, tmp2 = 0, tmp3 = 0;
	uint32_t pressure = 0;

	/* Calculate tmp1 */
	tmp1 = (conf->fine_temp >> 1) - 64000;
	/* Calculate tmp2 */
	tmp2 = (((tmp1 >> 2) * (tmp1 >> 2)) >> 11) * conf->cal.P6;
	tmp2 = tmp2 + ((tmp1 * conf->cal.P5) << 1);
	tmp2 = (tmp2 >> 2) + (conf->cal.P4 << 16);
	/* Update tmp1 */
	tmp3 = (conf->cal.P3 * (((tmp1 >> 2) * (tmp1 >> 2)) >> 13)) >> 3;
	tmp1 = (tmp3 + ((conf->cal.P2 * tmp1) >> 1)) >> 18;
	tmp1 = (((32768 + tmp1)) * (int)conf->cal.P1) >> 15;
	/* Calculate pressure */
	pressure = ((uint32_t)(1048576 - uncomp_pressure) - (tmp2 >> 12)) * 3125;

	/* Avoid exception caused by division by zero */
	if (tmp1 == 0) {
		return 0;
	}
	if (pressure < 0x80000000) {
		pressure = (pressure << 1) / ((uint32_t)tmp1);
	} else {
		pressure = (pressure / (uint32_t)tmp1) * 2;
	}

	tmp1 = (conf->cal.P9 * ((int)(((pressure >> 3) * (pressure >> 3)) >> 13))) >> 12;
	tmp2 = (((int)(pressure >> 2)) * conf->cal.P8) >> 13;
	pressure = (uint32_t)((int)pressure + ((tmp1 + tmp2 + conf->cal.P7) >> 4));

	return pressure;
}


/* Compute actual humidity from uncompensated humidity
 * Returns the value in 0.01 %rH
 * Output value of "4132" equals 41.32 %rH.
 */
uint32_t bme280_compensate_humidity(struct bme280_sensor_config* conf, int uncomp_humidity)
{
	int tmp1 = 0, tmp2 = 0, tmp3 = 0;
	uint32_t humidity = 0;

	/* Calculate tmp1 */
	tmp1 = conf->fine_temp - 76800;
	/* Calculate tmp2 */
	tmp2 = ((uncomp_humidity << 14) - (conf->cal.H4 << 20) - (conf->cal.H5 * tmp1) + 16384) >> 15;
	/* Calculate tmp3 */
	tmp3 = ((((tmp1 * conf->cal.H6) >> 10) * (((tmp1 * (int)conf->cal.H3) >> 11) + 32768)) >> 10) + 2097152;
	/* Update tmp1 */
	tmp1 = tmp2 * ((tmp3 * conf->cal.H2 + 8192) >> 14);
	tmp1 = tmp1 - (((((tmp1 >> 15) * (tmp1 >> 15)) >> 7) * (int)conf->cal.H1) >> 4);
	if (tmp1 < 0) {
		tmp1 = 0;
	}
	if (tmp1 > 419430400) {
		tmp1 = 419430400;
	}
	humidity = (uint32_t)(tmp1 >> 12);
	/* Convert from 32bit integer in Q22.10 format (22 integer 10 fractional bits) to a value
	 *   in 0.01 %rH :
	 * A value of 42313 represents 42313 / 1024 = 41.321 %rH, convert it to 4132, which is 41.32 %rH.
	 */
	humidity = ((humidity >> 10) * 100) + ((((humidity & 0x3FF) * 1000) >> 10) / 10);
	return humidity;

}



