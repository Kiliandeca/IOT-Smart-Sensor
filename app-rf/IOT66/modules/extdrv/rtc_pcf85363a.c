/****************************************************************************
 *   extdrv/rtc_pcf85363a.c
 *
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
#include "lib/string.h"
#include "lib/stdio.h"
#include "drivers/i2c.h"
#include "extdrv/rtc_pcf85363a.h"


/***************************************************************************** */
/*          Support for PCF85363A RTC from NXP                                 */
/***************************************************************************** */

enum rtc_pcf85363a_regs {
	/* Time registers */
	PCF85363A_TIME = 0x00,
	/* Alarm registers */
	/* Timestamp registers */
	PCF85363A_TSTMP_1 = 0x11,
	PCF85363A_TSTMP_2 = 0x17,
	PCF85363A_TSTMP_3 = 0x1D,
	PCF85363A_TSTMP_CTRL = 0x23,
	/* Offset register */
	PCF85363A_OFFSET = 0x24,
	/* Control registers */
	PCF85363A_CTRL_OSC = 0x25,
	PCF85363A_CTRL_BATT,
	PCF85363A_CTRL_PIN_IO,
	PCF85363A_CTRL_FUNC,
	PCF85363A_CTRL_INTA,
	PCF85363A_CTRL_INTB,
	PCF85363A_CTRL_FLAGS,
	/* Single RAM byte */
	PCF85363A_SINGLE_RAM = 0x2C,
	/* Watchdog */
	PCF85363A_WATCHDOG = 0x2D,
	/* Stop and reset */
	PCF85363A_STOP,
	PCF85363A_RESETS,
	/* RAM */
	PCF85363A_RAM_START = 0x40,
	PCF85363A_RAM_END = 0x7F,
};
#define RTC_RAM_SIZE  64

/* Time bits */
#define PCF85363A_TIME_OS       (0x01 << 7)  /* This one is in the "seconds" register */
#define PCF85363A_TIME_EMON     (0x01 << 7)  /* This one is in the "minutes" register */
#define PCF85363A_TIME_AMPM     (0x01 << 5)  /* This one is in the "hours" register */
/* Alarm enable bits */
/* Oscilator bits */
#define PCF85363A_OSC_CLKIV     (0x01 << 7)
#define PCF85363A_OSC_OFFM      (0x01 << 6)
#define PCF85363A_OSC_12_24     (0x01 << 5)
#define PCF85363A_OSC_LOWJ      (0x01 << 4)
#define PCF85363A_OSC_CLK_MASK  (PCF85363A_OSC_CLKIV | PCF85363A_OSC_LOWJ)
#define PCF85363A_OSC_OSCD(x)   (((x) & 0x03) << 2)
#define PCF85363A_OSC_LC(x)     (((x) & 0x03) << 0)
/* Battery control bits */
#define PCF85363A_BATT_BSOFF      (0x01 << 4)
#define PCF85363A_BATT_BSRR_HIGH  (0x01 << 3)
#define PCF85363A_BATT_BSM(x)     (((x) & 0x03) << 1)
#define PCF85363A_BATT_BSTH_2_8V  (0x01 << 0)
#define PCF85363A_BATT_MASK       0x1F
/* Pin IO controll bits */
#define PCF85363A_PIO_CLKPM_DIS (0x01 << 7)
#define PCF85363A_PIO_TSPULL    (0x01 << 6)
#define PCF85363A_PIO_TSL       (0x01 << 5)
#define PCF85363A_PIO_TSIM      (0x01 << 4)
#define PCF85363A_PIO_TSPM(x)   (((x) & 0x03) << 2)
 #define PCF85363A_PIO_INTB_DIS   0
 #define PCF85363A_PIO_INTB       1
 #define PCF85363A_PIO_INTB_CLK   2
 #define PCF85363A_PIO_INTB_INPUT 3
#define PCF85363A_PIO_INTAPM(x) (((x) & 0x03) << 0)
 #define PCF85363A_PIO_INTA_CLK   0
 #define PCF85363A_PIO_INTA_BATT  1
 #define PCF85363A_PIO_INTA       2
 #define PCF85363A_PIO_INTA_HI_Z  3
/* Function bits */
#define PCF85363A_FUNC_100TH    (0x01 << 7)
#define PCF85363A_FUNC_PI(x)    (((x) & 0x03) << 5)
#define PCF85363A_FUNC_RTCM     (0x01 << 4)
#define PCF85363A_FUNC_STOPM    (0x01 << 3)
#define PCF85363A_FUNC_COF(x)   (((x) & 0x07) << 0)
#define PCF85363A_FUNC_COF_MASK 0x07
/* Interrupt enable bits */
#define PCF85363A_INT_ILP       (0x01 << 7)
#define PCF85363A_INT_PIE       (0x01 << 6)
#define PCF85363A_INT_OIE       (0x01 << 5)
#define PCF85363A_INT_A1IE      (0x01 << 4)
#define PCF85363A_INT_A2IE      (0x01 << 3)
#define PCF85363A_INT_TSRIE     (0x01 << 2)
#define PCF85363A_INT_BSIE      (0x01 << 1)
#define PCF85363A_INT_WDIE      (0x01 << 0)
/* Interrupt flags bits */
#define PCF85363A_FLAGS_PI      (0x01 << 7)
#define PCF85363A_FLAGS_A2      (0x01 << 6)
#define PCF85363A_FLAGS_A1      (0x01 << 5)
#define PCF85363A_FLAGS_WD      (0x01 << 4)
#define PCF85363A_FLAGS_BS      (0x01 << 3)
#define PCF85363A_FLAGS_TSR3    (0x01 << 2)
#define PCF85363A_FLAGS_TSR2    (0x01 << 1)
#define PCF85363A_FLAGS_TSR1    (0x01 << 0)
/* Watchdog bits */
#define PCF85363A_WD_WDM        (0x01 << 7)
#define PCF85363A_WD_WDR(x)     (((x) & 0x0F) << 2)
#define PCF85363A_WD_WDS(x)     (((x) & 0x03) << 0)
/* Stop bits */
#define PCF85363A_STOP_BIT      (0x01 << 0)
/* Reset bits */
#define PCF85363A_RESET_CPR     (0x01 << 7)
#define PCF85363A_RESET_SR      (0x01 << 3)
#define PCF85363A_RESET_CTS     (0x01 << 0)
#define PCF85363A_RESET_MASK    (0x24)




#define REG_WR_CMD_SIZE 2
#define READ_CMD_SIZE 3
/* Let's consider that maximum write size is whole RAM */
#define MAX_WR_BUF_SIZE  (REG_WR_CMD_SIZE + RTC_RAM_SIZE)


/* Check RTC presence */
static int rtc_pcf85363_probe(struct rtc_pcf85363a_config* conf)
{
	char cmd_buf = (conf->addr | I2C_READ_BIT);

	/* Did we already probe the sensor ? */
	if (conf->probe_ok != 1) {
		conf->probe_ok = i2c_read(conf->bus_num, &cmd_buf, 1, NULL, NULL, 0);
	}
	return conf->probe_ok;
}

static int rtc_pcf85363_set_regs(struct rtc_pcf85363a_config* conf,
					uint8_t reg_start, uint8_t* values, uint8_t len)
{
	int ret = 0;
	char buf[MAX_WR_BUF_SIZE] = { conf->addr, };

	if (rtc_pcf85363_probe(conf) != 1) {
		return -ENODEV;
	}
	if (values == NULL) {
		return -EINVAL;
	}
	if ((reg_start + len - 1) > PCF85363A_RAM_END) {
		return -EINVAL;
	}
	buf[1] = reg_start;

	memcpy((buf + REG_WR_CMD_SIZE), values, len);
	ret = i2c_write(conf->bus_num, buf, (REG_WR_CMD_SIZE + len), NULL);
	if (ret != (REG_WR_CMD_SIZE + len)) {
		conf->probe_ok = 0;
		return ret;
	}
	return 0;
}
static int rtc_pcf85363_get_regs(struct rtc_pcf85363a_config* conf,
					uint8_t reg_start, uint8_t* values, uint8_t len)
{
	int ret = 0;
	char cmd_buf[READ_CMD_SIZE] = { conf->addr, 0, (conf->addr | I2C_READ_BIT), };
	char ctrl_buf[READ_CMD_SIZE] = { I2C_CONT, I2C_DO_REPEATED_START, I2C_CONT, };

	if (rtc_pcf85363_probe(conf) != 1) {
		return -ENODEV;
	}
	if (values == NULL) {
		return -EINVAL;
	}
	if ((reg_start + len - 1) > PCF85363A_RAM_END) {
		return -EINVAL;
	}
	cmd_buf[1] = reg_start;

	ret = i2c_read(conf->bus_num, cmd_buf, READ_CMD_SIZE, ctrl_buf, values, len);
	if (ret != len) {
		conf->probe_ok = 0;
		return ret;
	}
	return 0;
}


/* Time Read
 * When performing a time read operation, all time registers have to be read at once as
 *  a copy of the internal values is made by the device to a temporary buffer at the
 *  beginning of the read for data coherency.
 */
int rtc_pcf85363_time_read(struct rtc_pcf85363a_config* conf, struct rtc_time* time)
{
	int ret = 0;
	ret = rtc_pcf85363_get_regs(conf, PCF85363A_TIME, (uint8_t*)time, sizeof(struct rtc_time));
	if (ret != 0) {
		return ret;
	}
	/* Remove flags from time values */
	time->sec &= ~(PCF85363A_TIME_OS);
	time->min &= ~(PCF85363A_TIME_EMON);
	return 0;
}

/* Time Write
 * A time write operation has to be done in one go for all time registers for time data
 *  coherency.
 * Also, when performing a time write, the STOP bit must be set and the prescalers should
 *  be cleared.
 */
int rtc_pcf85363_time_write(struct rtc_pcf85363a_config* conf, struct rtc_time* time)
{
	int ret = 0;
	uint8_t stop_cmd_buf[2] = {
		PCF85363A_STOP_BIT,
		PCF85363A_RESET_CPR | PCF85363A_RESET_MASK,
	};
	if (time == NULL) {
		return -EINVAL;
	}
	/* Stop the RTC */
	ret = rtc_pcf85363_set_regs(conf, PCF85363A_STOP, stop_cmd_buf, 2);
	if (ret != 0) {
		return ret;
	}
	/* Set time */
	ret = rtc_pcf85363_set_regs(conf, PCF85363A_TIME, (uint8_t*)time, sizeof(struct rtc_time));
	if (ret != 0) {
		return ret;
	}
	/* And let RTC count again */
	stop_cmd_buf[0] = 0x00;
	ret = rtc_pcf85363_set_regs(conf, PCF85363A_STOP, stop_cmd_buf, 1);
	if (ret != 0) {
		return ret;
	}
	conf->time_set = 1;
	return 0;
}


/* Get one of the timestamps
 */
int rtc_pcf85363_get_timestamp(struct rtc_pcf85363a_config* conf,
									struct rtc_timestamp* tstmp, uint8_t timestamp_num)
{
	uint8_t addr = 0;
	if ((timestamp_num == 0) || (timestamp_num > 3)) {
		return -EINVAL;
	}

	addr = PCF85363A_TSTMP_1 + ((timestamp_num - 1) * sizeof(struct rtc_timestamp));
	return rtc_pcf85363_get_regs(conf, addr, (uint8_t*)tstmp, sizeof(struct rtc_timestamp));
}


/* Erase timestamps */
int rtc_pcf85363_erase_timestamps(struct rtc_pcf85363a_config* conf)
{
	uint8_t value = (PCF85363A_RESET_CTS | PCF85363A_RESET_MASK);
	return rtc_pcf85363_set_regs(conf, PCF85363A_RESETS, &value, 1);
}


/* Print the time to a usable string, in a unix format which "date" could understand. */
int rtc_pcf85363_time_to_str(struct rtc_time* time, char* str, int size)
{
	int len = 0;
	len = snprintf(str, size, "20%02x/%02x/%02x %02x:%02x:%02x (%02x)\n",
			time->year, time->month, time->day,
			time->hour, time->min, time->sec, time->weekday);
	return len;
}

/* Compare two times
 * Return 0 if they are equal, 1 if t1 > t2, and -1 if t1 < t2
 */
int rtc_pcf85363a_time_cmp(const struct rtc_time* t1, const struct rtc_time* t2)
{
	/* Check years first ... */
	if (t1->year > t2->year) return 1;
	if (t1->year < t2->year) return -1;
	/* Same year. Check months */
	if (t1->month > t2->month) return 1;
	if (t1->month < t2->month) return -1;
	/* Go on with days ... */
	if (t1->day > t2->day) return 1;
	if (t1->day < t2->day) return -1;
	/* Go on with hours ... */
	if (t1->hour > t2->hour) return 1;
	if (t1->hour < t2->hour) return -1;
	/* Minutes */
	if (t1->min > t2->min) return 1;
	if (t1->min < t2->min) return -1;
	/* Seconds */
	if (t1->sec > t2->sec) return 1;
	if (t1->sec < t2->sec) return -1;
	/* 100th of seconds not activated yet ... */
	if (t1->hundredth_sec > t2->hundredth_sec) return 1;
	if (t1->hundredth_sec < t2->hundredth_sec) return -1;
	/* Hey ... equal ! */
	return 0;
}

/* Check if RTC is configured and holds a valid time
 * This is done by reading the "SINGLE_RAM" byte, looking for PCF85363A_CONFIGURED,
 *  and checking that the date is after the oldest possible date given as parameter.
 * Returns -EFAULT if current time is older than "oldest" awaited time.
 * Returns 0 if device was not configured.
 * Returns 1 if device is configured and holds a valid time.
 */
int rtc_pcf85363a_is_up(struct rtc_pcf85363a_config* conf, const struct rtc_time* oldest)
{
	int ret = 0;
	uint8_t marker = 0;
	struct rtc_time now;

	/* Get configured status */
	ret = rtc_pcf85363_get_regs(conf, PCF85363A_SINGLE_RAM, &marker, 1);
	if (ret != 0) {
		return ret;
	} else if (marker != conf->config_marker) {
		return 0; /* Unconfigured */
	}

	/* Get current RTC time */
	ret =  rtc_pcf85363_time_read(conf, &now);
	if (ret != 0) {
		return ret;
	}
	/* Compare to oldest allowed time */
	if (rtc_pcf85363a_time_cmp(&now, oldest) <= 0) {
		return -EFAULT;
	}

	/* OK, up and running */
	conf->time_set = 1;
	return 1;
}


/* Read RAM memory */
int rtc_pcf85363a_read_ram(struct rtc_pcf85363a_config* conf,
								uint8_t offset, uint8_t* data, uint8_t len)
{
	if ((offset + len - 1) > RTC_RAM_SIZE) {
		return -EINVAL;
	}
	return rtc_pcf85363_get_regs(conf, (PCF85363A_RAM_START + offset), data, len);
}

/* Write RAM memory */
int rtc_pcf85363a_write_ram(struct rtc_pcf85363a_config* conf,
								uint8_t offset, uint8_t* data, uint8_t len)
{
	if ((offset + len - 1) > RTC_RAM_SIZE) {
		return -EINVAL;
	}
	return rtc_pcf85363_set_regs(conf, (PCF85363A_RAM_START + offset), data, len);
}



/* RTC config according to given structure fields
 * This configuration function relies on the value stored in the SINGLE_RAM
 *   byte to check for existing configuration.
 * Returns 1 if the RTC is considered as already configured, or 0 on configuration
 *   success.
 * Returns the error code of the failed configuration call upon errors.
 */
enum rtc_pcf85363a_cfg_regs {
	PCF85363A_CFG_TSTMP_CTRL = 0, /* 0x23 */
	PCF85363A_CFG_OFFSET,
	PCF85363A_CFG_CTRL_OSC,
	PCF85363A_CFG_CTRL_BATT,
	PCF85363A_CFG_CTRL_PIN_IO,
	PCF85363A_CFG_CTRL_FUNC,
	PCF85363A_CFG_CTRL_INTA,
	PCF85363A_CFG_CTRL_INTB,
	PCF85363A_CFG_CTRL_FLAGS,
	PCF85363A_CFG_SINGLE_RAM,
	PCF85363A_CFG_WATCHDOG,
	RTC_CONFIG_BUF_SIZE,
};
int rtc_pcf85363a_config(struct rtc_pcf85363a_config* conf)
{
	uint8_t config[RTC_CONFIG_BUF_SIZE];
	uint8_t marker = 0;
	int ret = 0;

	/* Get the configuration marker (SINGLE RAM byte) */
	ret = rtc_pcf85363_get_regs(conf, PCF85363A_SINGLE_RAM, &marker, 1);
	if (ret != 0) {
		return ret;
	} else if (marker == conf->config_marker) {
		return 1; /* Already configured */
	}
	/* Prepare configuration :
	 *  - timestamp control and offset set to 0x00
	 */
	memset(config, 0x00, RTC_CONFIG_BUF_SIZE);
	/* Oscilator */
	if (conf->mode & PCF85363A_MODE_12H) {
		config[PCF85363A_CFG_CTRL_OSC] |= PCF85363A_OSC_12_24;
	}
	if (conf->mode & PCF85363A_MODE_CLKOUT_ON) {
		config[PCF85363A_CFG_CTRL_OSC] |= (conf->clkout_ctrl & PCF85363A_OSC_CLK_MASK);
	}
	config[PCF85363A_CFG_CTRL_OSC] |= conf->oscilator_ctrl;
	/* Battery */
	config[PCF85363A_CFG_CTRL_BATT] |= (conf->batt_ctrl & PCF85363A_BATT_MASK);
	/* Pin IO */
	if ((conf->mode & PCF85363A_MODE_CLKOUT_ON) == 0) {
		config[PCF85363A_CFG_CTRL_PIN_IO] =
				PCF85363A_PIO_CLKPM_DIS | PCF85363A_PIO_INTAPM(PCF85363A_PIO_INTA_HI_Z);
	}
	config[PCF85363A_CFG_CTRL_PIN_IO] |= 0x00;
	/* Function */
	if (conf->mode & PCF85363A_MODE_100TH) {
		config[PCF85363A_CFG_CTRL_FUNC] |= PCF85363A_FUNC_100TH;
	}
	if (conf->mode & PCF85363A_MODE_STOP_WATCH) {
		config[PCF85363A_CFG_CTRL_FUNC] |= PCF85363A_FUNC_RTCM;
	}
	if (conf->mode & PCF85363A_MODE_CLKOUT_ON) {
		config[PCF85363A_CFG_CTRL_FUNC] |= (conf->clkout_ctrl & PCF85363A_FUNC_COF_MASK);
	}
	/* Mark the device as configured */
	config[PCF85363A_CFG_SINGLE_RAM] = conf->config_marker;
	/* Reset flags */
	config[PCF85363A_CFG_CTRL_FLAGS] = 0x00;

	/* Perform configuration */	
	ret = rtc_pcf85363_set_regs(conf, PCF85363A_TSTMP_CTRL, config, RTC_CONFIG_BUF_SIZE);
	if (ret != 0) {
		return ret;
	}
	return 0; /* Config success */
}

