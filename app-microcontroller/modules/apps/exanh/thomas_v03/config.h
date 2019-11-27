/****************************************************************************
 *   apps/exanh/thomas_v03/config.h
 *
 * RobotGrow main board support
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

/* ISP button release disables the chenillard for 5 seconds */

#ifndef ROBOTGROW_CONFIG_H
#define ROBOTGROW_CONFIG_H

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "lib/errno.h"
#include "lib/utils.h"
#include "drivers/adc.h"
#include "drivers/ssp.h"
#include "extdrv/status_led.h"
#include "drivers/timers.h"

#include "extdrv/ws2812.h"
#include "extdrv/tmp101_temp_sensor.h"
#include "extdrv/rtc_pcf85363a.h"

#include "extdrv/sdmmc.h"

#include "extdrv/chirp.h"



#define SELECTED_FREQ  FREQ_SEL_48MHz


/***************************************************************************** */
/* Pins configuration */

extern const struct pio isp;   /* ISP button */
extern const struct pio button; /* User button */

extern const struct pio pompe_on; /* Water Pump */


/***************************************************************************** */
/* SD/MMC Card */
#define MMC_BUF_SIZE MMC_MAX_SECTOR_SIZE
#define MMC_RECORD_SIZE  32
extern uint16_t mmc_idx;
extern uint32_t mmc_block_num;
extern uint8_t mmc_data[MMC_BUF_SIZE];
#define MMC_BLK_VALID_MAGIC  "This is a valid data block" /* Must be less than 32 */
#define MMC_BLK_VALID_IDX_MAGIC "ValidIDX"
#define MMC_BLK_VALID_IDX_MAGIC_SIZE 8
#define MMC_LAST_DATA_IDX_DATE_OFFSET  MMC_BLK_VALID_IDX_MAGIC_SIZE
#define MMC_LAST_DATA_IDX_OFFSET  10  /* 10 * 4 bytes */
#define MMC_LAST_DATA_IDX_BLOCK   1

extern struct sdmmc_card micro_sd;

/***************************************************************************** */
/* RTC and time */
extern struct rtc_pcf85363a_config rtc_conf;
extern struct rtc_time now;
extern struct rtc_time time_update;
extern volatile uint8_t perform_time_update;
extern int time_valid;


/***************************************************************************** */
/* TMP101 onboard I2C temperature sensor */
extern struct tmp101_sensor_config tmp101_sensor;

/***************************************************************************** */
/* Chirp sensor */
extern struct chirp_sensor_config chirp;



/***************************************************************************** */
/* From config.c */

void system_init(void);

void system_config(void);


/***************************************************************************** */
/* From main.c */

/* Callbacks */
void request_samples(uint32_t tick);
void pompe_force(uint32_t gpio);
void decrementer(uint32_t tick);
void pump_manage(uint32_t tick);

void handle_cmd(uint8_t c);

#endif /* ROBOTGROW_CONFIG_H */

