/****************************************************************************
 *   apps/exanh/thomas_v03/config.c
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

#include "config.h"


/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* ADC */
	{ LPC_ADC_AD1_PIO_0_31,  LPC_IO_ANALOG }, /* ADC1 : External sensor */
	{ LPC_ADC_AD4_PIO_1_2,  LPC_IO_ANALOG },   /* ADC2 : Battery */
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_15, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* SD Card CS */
	/* GPIO */
	{ LPC_GPIO_0_0, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Led strip */
	{ LPC_GPIO_0_4, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Humidity Sensors Power control */
	{ LPC_GPIO_0_12, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* ISP Button */
	{ LPC_GPIO_0_23, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Pompe */
	{ LPC_GPIO_1_1, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Button */
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio isp = LPC_GPIO_0_12;   /* ISP button */
const struct pio button = LPC_GPIO_1_1; /* User button */

const struct pio ws2812_data_out_pin = LPC_GPIO_0_0; /* Led control data pin */

const struct pio pompe_on = LPC_GPIO_0_23; /* Water Pump */


/***************************************************************************** */
/* SD/MMC Card */
struct sdmmc_card micro_sd = {
	.ssp_bus_num = SSP_BUS_0,
	.card_type = MMC_CARDTYPE_UNKNOWN,
	.block_size = 512,
	.chip_select = LPC_GPIO_0_15,
};
uint16_t mmc_idx = 0;
uint32_t mmc_block_num = 0;
uint8_t mmc_data[MMC_BUF_SIZE];


/***************************************************************************** */
/* RTC and time */
#define RTC_ADDR   0xA2
struct rtc_pcf85363a_config rtc_conf = {
	.bus_num = I2C0,
	.addr = RTC_ADDR,
	.mode = PCF85363A_MODE_RTC,
	.config_marker = PCF85363A_CONFIGURED_1,
	.batt_ctrl = PCF85363A_CONF_BATT_TH_2_8V,
};
/* Oldest acceptable time in RTC. BCD coded. */
const struct rtc_time oldest = {
	.year = 0x17,
	.month = 0x08,
	.day = 0x31,
	.hour = 0x13,
	.min = 0x37,
};
struct rtc_time now;
volatile uint8_t perform_time_update = 0;
struct rtc_time time_update;
int time_valid = 0;


/***************************************************************************** */
/* TMP101 onboard I2C temperature sensor */
#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};


/***************************************************************************** */
/* Chirp sensor */
#define CHIRP_ADDR  0x40 /* On 8 bits */
struct chirp_sensor_config chirp = {
	.bus_num = I2C0,
	.addr = CHIRP_ADDR,
};



/***************************************************************************** */
void system_init(void)
{
	/* Core */
	startup_watchdog_disable(); /* Stop the watchdog right now, before it gets a chance to break in */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	status_led_config(&status_led_green, &status_led_red);
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();
	/* Internal drivers */
	uart_on(UART0, 115200, handle_cmd);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	ssp_master_on(SSP_BUS_0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);
	adc_on(NULL);
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	uprintf(UART0, name);
	while (1);
}



/***************************************************************************** */
void system_config(void)
{
	int ret = 0;

	/* Configure temperature sensor */
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(UART0, "Temp config error: %d\n", ret);
	}

	/* GPIO for Pump control */
	config_gpio(&pompe_on, 0, GPIO_DIR_OUT, 0);
	set_gpio_callback(pompe_force, &button, EDGES_BOTH);

	/* WS2812B Leds */
	ws2812_config(&ws2812_data_out_pin);

	/* RTC init */
	ret = rtc_pcf85363a_config(&rtc_conf);
	ret = rtc_pcf85363a_is_up(&rtc_conf, &oldest);
	if (ret == 1) {
		char buff[30];
		rtc_pcf85363_time_read(&rtc_conf, &now);
		rtc_pcf85363_time_to_str(&now, buff, 30);
		/* Debug */
		uprintf(UART0, "Using time from RTC: %s\n", buff);
	} else if (ret == -EFAULT) {
		char buff[30];
		memcpy(&now, &oldest, sizeof(struct rtc_time));
		rtc_pcf85363_time_write(&rtc_conf, &now);
		rtc_pcf85363_time_to_str(&now, buff, 30);
		uprintf(UART0, "Using time from source code as now : %s\n", buff);
		time_valid = 0;
	}

	/* Chirp init */
	ret = chirp_reset(&chirp);
	if (ret < 0) {
		uprintf(UART0, "No chirp detected\n");
	} else {
		uprintf(UART0, "Chirp detected, reset OK.\n");
	}

	/* microSD card init */
	do {
		ret = sdmmc_init(&micro_sd);
		if (ret == 0) {
			msleep(5);
			ret = sdmmc_init_wait_card_ready(&micro_sd);
			if (ret == 0) {
				ret = sdmmc_init_end(&micro_sd);
			}
		}
		uprintf(UART0, "uSD init: %d, type: %d, bs: %d\n", ret, micro_sd.card_type, micro_sd.block_size);
		msleep(50);
	} while (ret != 0);
	ret = sdmmc_read_block(&micro_sd, 0, mmc_data);
	uprintf(UART0, "uSD read: %s\n", mmc_data);
	/* Find the next empty block */
	/* Read the second block and get the date and block number */
	ret = sdmmc_read_block(&micro_sd, MMC_LAST_DATA_IDX_BLOCK, mmc_data);
	if (strncmp((char*)mmc_data, MMC_BLK_VALID_IDX_MAGIC, MMC_BLK_VALID_IDX_MAGIC_SIZE) != 0) {
		/* Never wrote a valid block number ? */
		mmc_block_num = MMC_LAST_DATA_IDX_BLOCK;
	} else {
		mmc_block_num = ((uint32_t*)mmc_data)[MMC_LAST_DATA_IDX_OFFSET];
	}
	/* Search for the last block from this one */
	do {
		mmc_block_num++;
		ret = sdmmc_read_block(&micro_sd, mmc_block_num, mmc_data);
		if (ret != 0) {
			uprintf(UART0, "SD Read failed while looking for last block at block %d (ret = %d)\n", mmc_block_num, ret);
		}
	} while (strncmp((char*)mmc_data, MMC_BLK_VALID_MAGIC, (sizeof(MMC_BLK_VALID_MAGIC) - 1)) == 0);
	/* Debug */
	uprintf(UART0, "Last valid data block on uSD is %d\n", mmc_block_num);

	/* Set the date to the last found date */
	ret = sdmmc_read_block(&micro_sd, (mmc_block_num - 1), mmc_data);
	if (rtc_pcf85363a_time_cmp(&now, (struct rtc_time*)(mmc_data + MMC_BLK_VALID_IDX_MAGIC_SIZE)) < 0) {
		char buff[30];
		memcpy(&now, (struct rtc_time*)(mmc_data + MMC_BLK_VALID_IDX_MAGIC_SIZE), sizeof(struct rtc_time));
		rtc_pcf85363_time_write(&rtc_conf, &now);
		rtc_pcf85363_time_to_str(&now, buff, 30);
		uprintf(UART0, "Using last saved time as now : %s\n", buff);
		time_valid = 0;
	}

	/* Prepare buffer for first set of data */
	if (1) {
		int offset = 0;
		/* Start with magic */
		memcpy(mmc_data, MMC_BLK_VALID_MAGIC, sizeof(MMC_BLK_VALID_MAGIC));
		offset = 32;
		/* Add date */
		rtc_pcf85363_time_read(&rtc_conf, &now);
		memcpy((mmc_data + offset), &now, sizeof(struct rtc_time));
		offset += sizeof(struct rtc_time);
		/* Internal temp and battery voltage are not available ... skip. */
		mmc_idx = 64;
	}

	/* Register callback to send samples requests to sensors.
	 * Send request to one sensor every 10 seconds, which makes 1 request every
	 * minute for each sensor.
	 */
	add_systick_callback(request_samples, 10*1000);
	add_systick_callback(pump_manage, 10*1000);
	add_systick_callback(decrementer, 10*1000);

	uprintf(UART0, "E-Xanh Gardener started\n");
}


