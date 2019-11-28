/****************************************************************************
 *   extdrv/rtc_pcf85363a.h
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

#ifndef EXTDRV_RTC_PCF85363A
#define EXTDRV_RTC_PCF85363A

/* This driver actually has support only for the main time registers in RTC mode.
 */

struct rtc_time {
	uint8_t hundredth_sec; /* 0 to 99 - BCD encoding */
	uint8_t sec; /* 0 to 59 - BCD encoding */
	uint8_t min; /* 0 to 59 - BCD encoding */
	uint8_t hour; /* 0 to 24 or AM/PM + 0 to 12 - BCD encoding */
	uint8_t day; /* 1 to 31 - BCD encoding */
	uint8_t weekday; /* 0 to 6 - BCD encoding */
	uint8_t month; /* 1 to  12 - BCD encoding */
	uint8_t year; /* 0 to 99 - BCD encoding */
};

struct rtc_timestamp {
	uint8_t sec; /* 0 to 59 - BCD encoding */
	uint8_t min; /* 0 to 59 - BCD encoding */
	uint8_t hour; /* 0 to 24 or AM/PM + 0 to 12 - BCD encoding */
	uint8_t day; /* 1 to 31 - BCD encoding */
	uint8_t month; /* 1 to  12 - BCD encoding */
	uint8_t year; /* 0 to 99 - BCD encoding */
};



struct rtc_pcf85363a_config {
	uint8_t bus_num; /* I2C bus number */
	uint8_t addr; /* Address on I2C bus. 8bit format */
	/* probe_ok is set to 1 once RTC got probed, reset to 0 upon communication
	 *  errors */
	uint8_t probe_ok;
	/* time_set is set to 1 once time got set or time is found to be newer than
	 *  the date given on rtc_pcf85363a_is_up() call. */
	uint8_t time_set;

	/* config_marker should be set to one of the PCF85363A_CONFIGURED_* values
	 *  and changed whenever the configuration is changed within the program in
	 *  order to perform re-configuration only on configuration changes */
#define PCF85363A_CONFIGURED_0   0x5A
#define PCF85363A_CONFIGURED_1   0x78
#define PCF85363A_CONFIGURED_2   0x33
	uint8_t config_marker;

	/* mode is a bit mask of mode settings
	 * Concerned bits :
	 *  - 12_24 from PCF85363A_CTRL_OSC
	 *  - CLKPM from PCF85363A_CTRL_PIN_IO
	 *  - 100TH and RTCM from PCF85363A_CTRL_FUNC
	 * Default (0) sets RTC mode, 24 hours, and clkout off.
	 */
#define PCF85363A_MODE_RTC         (0x00 << 0)
#define PCF85363A_MODE_STOP_WATCH  (0x01 << 0)
#define PCF85363A_MODE_12H         (0x01 << 1)
#define PCF85363A_MODE_CLKOUT_ON   (0x01 << 2)
#define PCF85363A_MODE_100TH       (0x01 << 3)
	uint8_t mode;

	/* Control of oscilator.
	 * Concerned bits :
	 *  - OSCD[1:0] and CL[1:0] from PCF85363A_CTRL_OSC
	 */
#define PCF85363A_CONF_OSC_OSCD(x)   (((x) & 0x03) << 2)
 #define PCF85363A_OSC_NORMAL_DRIVE 0
 #define PCF85363A_OSC_LOW_DRIVE    1
 #define PCF85363A_OSC_HIGH_DRIVE   2
#define PCF85363A_CONF_OSC_LC(x)     (((x) & 0x03) << 0)
 #define PCF85363A_OSC_CAP_7pF    0
 #define PCF85363A_OSC_CAP_6pF    1
 #define PCF85363A_OSC_CAP_12pF   2
	uint8_t oscilator_ctrl;

	/* Control of clk out features.
	 * Concerned bits :
	 *  - CLKIV and LOWJ from PCF85363A_CTRL_OSC : bits 7 and 4
	 *  - COF[2:0] from PCF85363A_CTRL_FUNC : bits 0 to 2
	 */
#define PCF85363A_CONF_CLK_INV     (0x01 << 7)
#define PCF85363A_CONF_CLK_LOWJ    (0x01 << 4)
#define PCF85363A_CONF_CLK_FREQ(x) (((x) & 0x07) << 0)
 #define PCF85363A_CLKOUT_32KHZ 0
 #define PCF85363A_CLKOUT_16KHZ 1
 #define PCF85363A_CLKOUT_8KHZ  2
 #define PCF85363A_CLKOUT_4KHZ  3
 #define PCF85363A_CLKOUT_2KHZ  4
 #define PCF85363A_CLKOUT_1KHZ  5
 #define PCF85363A_CLKOUT_1HZ   6
 #define PCF85363A_CLKOUT_FIXED 7
	uint8_t clkout_ctrl;

	/* Control of Battery switching
	 * Concerned bits :
	 *  - All of PCF85363A_CTRL_BATT
	 *  - BSIEA and BSIEB from PCF85363A_CTRL_INTA and PCF85363A_CTRL_INTB
	 * Default (0) is Battery switch ON at low refresh rate, switching at Vth level
	 *  and Vth of 1.5V
	 */
#define PCF85363A_CONF_BATT_SWITCH_OFF (0x01 << 4)
#define PCF85363A_CONF_BATT_ST_REFRESH_HIGH  (0x01 << 3)
#define PCF85363A_CONF_BATT_TH_1_5V    (0x00 << 0)
#define PCF85363A_CONF_BATT_TH_2_8V    (0x01 << 0)
	uint8_t batt_ctrl;

	/* Pin "INTB" */
	uint8_t pin_inta;

	/* Pin "INTB" */
	uint8_t pin_intb;

	/* Interrupts */
	uint8_t interrupt_inta;
	uint8_t interrupt_intb;

	/* Watchdog
	 * Reset or "0" defaults to watchdog off
	 */
	uint8_t watchdog;
	
};


/* Time Read
 * When performing a time read operation, all time registers have to be read at once as
 *  a copy of the internal values is made by the device to a temporary buffer at the
 *  beginning of the read for data coherency.
 */
int rtc_pcf85363_time_read(struct rtc_pcf85363a_config* conf, struct rtc_time* time);

/* Time Write
 * A time write operation has to be done in one go for all time registers for time data
 *  coherency.
 * Also, when performing a time write, the STOP bit must be set and the prescalers should
 *  be cleared.
 */
int rtc_pcf85363_time_write(struct rtc_pcf85363a_config* conf, struct rtc_time* time);



/* Get one of the timestamps
 */
int rtc_pcf85363_get_timestamp(struct rtc_pcf85363a_config* conf,
									struct rtc_timestamp* tstmp, uint8_t timestamp_num);
/* Erase timestamps */
int rtc_pcf85363_erase_timestamps(struct rtc_pcf85363a_config* conf);



/* Print the time to a usable string, in a unix format which "date" could understand. */
int rtc_pcf85363_time_to_str(struct rtc_time* time, char* str, int size);

/* Compare two times
 * Return 0 if they are equal, 1 if t1 > t2, and -1 if t1 < t2
 */
int rtc_pcf85363a_time_cmp(const struct rtc_time* t1, const struct rtc_time* t2);


/* Read RAM memory */
int rtc_pcf85363a_read_ram(struct rtc_pcf85363a_config* conf,
								uint8_t offset, uint8_t* data, uint8_t len);
/* Write RAM memory */
int rtc_pcf85363a_write_ram(struct rtc_pcf85363a_config* conf,
								uint8_t offset, uint8_t* data, uint8_t len);



/* Check if RTC is configured and holds a valid time
 * This is done by reading the "SINGLE_RAM" byte, looking for PCF85363A_CONFIGURED,
 *  and checking that the date is after the oldest possible date given as parameter.
 * Returns -EFAULT if current time is older than "oldest" awaited time.
 * Returns 0 if device was not configured.
 * Returns 1 if device is configured and holds a valid time.
 */
int rtc_pcf85363a_is_up(struct rtc_pcf85363a_config* conf, const struct rtc_time* oldest);

/* RTC config according to given structure fields
 * This configuration function relies on the value stored in the SINGLE_RAM
 *   byte to check for existing configuration.
 * Returns 1 if the RTC is considered as already configured, or 0 on configuration
 *   success.
 * Returns the error code of the failed configuration call upon errors.
 */ 
int rtc_pcf85363a_config(struct rtc_pcf85363a_config* conf);


#endif /* EXTDRV_RTC_PCF85363A */
