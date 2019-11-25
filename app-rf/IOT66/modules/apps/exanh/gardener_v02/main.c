/****************************************************************************
 *   apps/exanh/gardener/main.c
 *
 * E-Xanh Gardener main board support
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

/* ISP button release disables the chenillard for 5 seconds */

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



#define MODULE_VERSION   0x02
#define MODULE_NAME "E-Xanh Gardener"


#define SELECTED_FREQ  FREQ_SEL_48MHz

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
	{ LPC_GPIO_0_6, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Water level switch input */
	{ LPC_GPIO_0_12, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* ISP Button */
	{ LPC_GPIO_0_20, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Motor Enable / Sleep */
	{ LPC_GPIO_0_21, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Motor Direction control */
	{ LPC_GPIO_0_22, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Motor ON */
	{ LPC_GPIO_0_23, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Pompe */
	{ LPC_GPIO_0_24, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 0 */
	{ LPC_GPIO_0_25, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 1 */
	{ LPC_GPIO_0_26, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 2 */
	{ LPC_GPIO_0_27, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 3 */
	{ LPC_GPIO_0_28, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 4 */
	{ LPC_GPIO_0_29, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 5 */
	{ LPC_GPIO_1_1, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Button */
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio isp = LPC_GPIO_0_12;   /* ISP button */
const struct pio button = LPC_GPIO_1_1; /* User button */

const struct pio water_level = LPC_GPIO_0_6; /* Water level info */

const struct pio ws2812_data_out_pin = LPC_GPIO_0_0; /* Led control data pin */

const struct pio motor_power = LPC_GPIO_0_22; /* Motor Power */
const struct pio motor_enable = LPC_GPIO_0_20; /* Motor Enable */
const struct pio motor_direction = LPC_GPIO_0_21; /* Motor Direction */

const struct pio pompe_on = LPC_GPIO_0_23; /* Water Pump */


#define NB_POS_SENSORS   6
const struct pio positions[NB_POS_SENSORS] = {
	LPC_GPIO_0_24,
	LPC_GPIO_0_25,
	LPC_GPIO_0_26,
	LPC_GPIO_0_27,
	LPC_GPIO_0_28,
	LPC_GPIO_0_29,
};


/***************************************************************************** */
/* SD/MMC Card */
struct sdmmc_card micro_sd = {
	.ssp_bus_num = SSP_BUS_0,
	.card_type = MMC_CARDTYPE_UNKNOWN,
	.block_size = 512,
	.chip_select = LPC_GPIO_0_15,
};
#define MMC_BUF_SIZE MMC_MAX_SECTOR_SIZE
#define MMC_RECORD_SIZE  32
static uint16_t mmc_idx = 0;
static uint32_t mmc_block_num = 0;
uint8_t mmc_data[MMC_BUF_SIZE];
#define MMC_BLK_VALID_MAGIC  "This is a valid data block" /* Must be less than 32 */
#define MMC_BLK_VALID_IDX_MAGIC "ValidIDX"
#define MMC_BLK_VALID_IDX_MAGIC_SIZE 8
#define MMC_LAST_DATA_IDX_DATE_OFFSET  MMC_BLK_VALID_IDX_MAGIC_SIZE
#define MMC_LAST_DATA_IDX_OFFSET  10  /* 10 * 4 bytes */
#define MMC_LAST_DATA_IDX_BLOCK   1

const struct pio sclk_error = LPC_GPIO_0_18; /* Fix for v0.2 of the board */


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
	.month = 0x05,
	.day = 0x16,
	.hour = 0x00,
	.min = 0x57,
};
static struct rtc_time now;
static volatile uint8_t perform_time_update = 0;
static struct rtc_time time_update;
static int time_valid = 0;


/***************************************************************************** */
/* TMP101 onboard I2C temperature sensor */
#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};


/***************************************************************************** */
#define MAX_SENSORS 6
volatile uint8_t poll_sensor = 0;
void request_samples(uint32_t tick)
{
	static uint8_t next_sensor = 1;
	poll_sensor = next_sensor++;
	if (next_sensor > MAX_SENSORS) {
		next_sensor = 1;
	}
}


/***************************************************************************** */
void system_init(void)
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	system_brown_out_detection_config(0); /* No ADC used */
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	status_led_config(&status_led_green, &status_led_red);
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();
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


static void pos_detect(uint32_t gpio);
static void water_level_info(uint32_t gpio);
static void pompe_force(uint32_t gpio);
static void motor_force(uint32_t gpio);

void system_config(void)
{
	int ret = 0;
	int i = 0;

	/* Configure temperature sensor */
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(UART0, "Temp config error: %d\n", ret);
	}

	/* GPIO for Pump control */
	config_gpio(&pompe_on, 0, GPIO_DIR_OUT, 0);
	set_gpio_callback(pompe_force, &button, EDGES_BOTH);

	/* Selector position sensors */
	for (i = 0; i < NB_POS_SENSORS; i++) {
		set_gpio_callback(pos_detect, &(positions[i]), EDGES_BOTH);
	}
	set_gpio_callback(water_level_info, &water_level, EDGES_BOTH);

	/* GPIO for Motor control */
	config_gpio(&motor_power, 0, GPIO_DIR_OUT, 0);
	config_gpio(&motor_enable, 0, GPIO_DIR_OUT, 0);
	config_gpio(&motor_direction, 0, GPIO_DIR_OUT, 0);
	set_gpio_callback(motor_force, &isp, EDGES_BOTH);

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

	/* microSD card init */
	/* FIX for v02 */
	config_gpio(&sclk_error, 0, GPIO_DIR_IN, 0);
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

	uprintf(UART0, "E-Xanh Gardener started\n");
}

/***************************************************************************** */
volatile uint8_t current_position = 255;
static void pos_detect(uint32_t gpio)
{
	if (gpio_read(positions[(gpio - 24)]) == 0) {
		/* The sensor output is 0 when it detects a magnet */
		current_position = gpio - 24;
	} else {
		current_position = gpio - 14;
	}
}
volatile uint8_t water_info = 0;
static void water_level_info(uint32_t gpio)
{
	if (gpio_read(water_level) == 0) {
		water_info = 0;
	} else {
		water_info = 1;
	}
}

#define ENV_BUFF_SIZE  20
uint8_t env_buff[ENV_BUFF_SIZE];
volatile int idx = 0;
volatile int mode = 0;
void handle_cmd(uint8_t c)
{
	if (c == '#') {
		mode = 1; /* Receiving data from sensors */
		env_buff[idx++] = c;
	} else if (c == '$') {
		mode = 2; /* Configuration */
	} else if (mode == 1) {
		if (idx < ENV_BUFF_SIZE) {
			env_buff[idx++] = c;
		}
		if (idx >= ENV_BUFF_SIZE) {
			mode = 0;
		}
	} else if (mode == 2) {
		if (c == 'c') {
			/* Received config request ? */
			/* FIXME */
			mode = 0;
		} else if (c == 'd') {
			/* Incoming date config */
			mode = 3;
		}
	} else if (mode == 3) {
		static uint8_t tidx = 0;
		((uint8_t*)(&time_update))[tidx++] = c;
		if (tidx >= sizeof(struct rtc_time)) {
			tidx = 0;
			perform_time_update = 1;
			mode = 0;
		}
	} else {
		/* Echo ? */
		if (0) {
			serial_send_quickbyte(UART0, c);
		}
	}

}
volatile uint8_t motor_sens = 2;
volatile uint8_t motor_move_req = 0;
static void motor_force(uint32_t gpio)
{
	static int next_motor_sens = 1;
	if (gpio_read(isp) == 0) {
		motor_sens = next_motor_sens++;
		if (next_motor_sens > 2) {
			next_motor_sens = 1;
		}
		motor_move_req = 1;
	} else {
		motor_move_req = 0;
	}
}
volatile uint8_t pompe_cmd = 0;
static void pompe_force(uint32_t gpio)
{
	if (gpio_read(button) == 0) {
		pompe_cmd = 1;
	} else {
		pompe_cmd = 0;
	}
}

/***************************************************************************** */
int main(void)
{
	uint8_t old_position = 255;
	uint8_t old_water_level = 255;
	uint8_t old_pompe = 0;
	int tmp101_deci_degrees = 0;
	int mv_batt = 0;

	system_init();

	uart_on(UART0, 115200, handle_cmd);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	ssp_master_on(SSP_BUS_0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);
	adc_on(NULL);

	system_config();

	while (1) {
		/* Request a temperature conversion to onboard sensor */
		tmp101_sensor_start_conversion(&tmp101_sensor);

		/* Add some delay to the loop */
		chenillard(100);

		/* Track selector positions */
		if (current_position != old_position) {
			uprintf(UART0, "New position : %d\n", current_position);
			old_position = current_position;
		}

		/* Track water level */
		if (water_info != old_water_level) {
			old_water_level = water_info;
			uprintf(UART0, "Water is at %d\n", water_info);
		}

		/* Pump activation ? */
		if (old_pompe != pompe_cmd) {
			/* Add data to the next mmc data block to be written */
			rtc_pcf85363_time_read(&rtc_conf, &now);
			memcpy((mmc_data + mmc_idx), &now, sizeof(struct rtc_time));
			if (pompe_cmd == 1) {
				memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time)), "Pump ON", 8);
				uprintf(UART0, "Pump On\n");
				gpio_set(pompe_on);
			} else {
				memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time)), "Pump OFF", 9);
				uprintf(UART0, "Pump Stopped\n");
				gpio_clear(pompe_on);
			}
			mmc_idx += 32;
			old_pompe = pompe_cmd;
		}

		/* Selector motor activation ? */
		if (motor_move_req == 1) {
			if (motor_sens == 1) {
				uprintf(UART0, "Motor moving forward\n");
				gpio_set(motor_direction);
			} else {
				uprintf(UART0, "Motor moving backward\n");
				gpio_clear(motor_direction);
			}
			msleep(1);
			/* Turn motor ON */
			gpio_set(motor_enable);
			gpio_set(motor_power);
		} else {
			/* turn motor back Off */
			gpio_clear(motor_power);
			gpio_clear(motor_enable);
		}

		/* Read onboard Temperature sensor */
		if (1) {
			int ret = 0;
			ret = tmp101_sensor_read(&tmp101_sensor, NULL, &tmp101_deci_degrees);
			if (ret != 0) {
				uprintf(UART0, "TMP101 read error : %d\n", ret);
			}
		}

		/* Send the poll request to one of the sensors */
		if (poll_sensor != 0) {
			uint8_t req_buf[4];
			req_buf[0] = '#';
			req_buf[1] = poll_sensor;
			poll_sensor = 0;
			req_buf[2] = 'a';
			serial_write(UART0, (char*)req_buf, 3);
		}

		/* Update led colors ? */
		if (0) {
			/* Set led */
			ws2812_set_pixel(0, 0, 10, 10);
			ws2812_set_pixel(1, 10, 0, 10);
			ws2812_send_frame(0);
		}

		/* Read onboard ADC for external analog sensor */
		if (0) {
			uint16_t humidity_val = 0;
			int mv_humidity = 0;
			adc_start_convertion_once(LPC_ADC(1), LPC_ADC_SEQ(0), 0);
			msleep(8);
			adc_get_value(&humidity_val, LPC_ADC(1));
			mv_humidity = ((humidity_val * 32) / 10);
			uprintf(UART0, "External sensor : %d\n", mv_humidity);
		}

		/* Read onboard ADC for battery voltage */
		if (1) {
			uint16_t batt_val = 0;
			adc_start_convertion_once(LPC_ADC(4), LPC_ADC_SEQ(0), 0);
			msleep(8);
			adc_get_value(&batt_val, LPC_ADC(4));
			mv_batt = (batt_val * 16);  /*  = ((val * 32) / 10) * 5  (pont diviseur) */
		}

		/* If we received data from a sensor, then add to SD buffer for storage and display */
		if ((idx >= ENV_BUFF_SIZE) && (mmc_idx <= (MMC_BUF_SIZE - MMC_RECORD_SIZE))) {
			uint16_t* data = (uint16_t*)env_buff;
			int i = 0;
			/* Add data to the next mmc data block to be written */
			rtc_pcf85363_time_read(&rtc_conf, &now);
			memcpy((mmc_data + mmc_idx), &now, sizeof(struct rtc_time));
			memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time)), env_buff, ENV_BUFF_SIZE);
			mmc_idx += 32;
			/* Switch data to our endianness */
			for (i = 1; i <= 8; i++) {
				data[i] = (uint16_t)ntohs(data[i]);
			}
			uprintf(UART0, "From sensor %d:\n", (env_buff[1] & 0x1F));
			uprintf(UART0, "- Soil: %d\n", data[1]);
			uprintf(UART0, "- Lux: %d, IR: %d, UV: %d\n", data[2], data[3], data[4]);
			uprintf(UART0, "- Patm: %d hPa, Temp: %d,%02d degC, Humidity: %d,%d rH\n\n",
							data[5],
							data[6] / 10,  (data[6]> 0) ? (data[6] % 10) : ((-data[6]) % 10),
							data[7] / 10, data[7] % 10);
			idx = 0;
			/* Data got received, clear sensor led. */
			env_buff[1] &= 0x1F;
			env_buff[2] = 'l';
			env_buff[3] = 0;
			env_buff[4] = 0;
			env_buff[5] = 0;
			serial_write(UART0, (char*)env_buff, 6);
		}

		/* If MMC buffer is full, then write it to the uSD card */
		if (mmc_idx > (MMC_BUF_SIZE - MMC_RECORD_SIZE)) {
			int ret = 0;
			/* Save data to uSD card */
			ret = sdmmc_write_block(&micro_sd, mmc_block_num, mmc_data);
			if (ret == 0) {
				uprintf(UART0, "Wrote data to block %d\n", mmc_block_num);
			} else {
				uprintf(UART0, "Error while writting data to block %d: %d\n", mmc_block_num, ret);
			}
			memset(mmc_data, 0, MMC_BUF_SIZE);
			/* Maybe update the "last block written index" */
			/* FIXME : Set to 0x3F */
			if ((mmc_block_num & 0x0F) == 0x00) {
				/* Prepare the block number 2 */
				memcpy(mmc_data, MMC_BLK_VALID_IDX_MAGIC, MMC_BLK_VALID_IDX_MAGIC_SIZE);
				rtc_pcf85363_time_read(&rtc_conf, &now);
				memcpy((mmc_data + MMC_LAST_DATA_IDX_DATE_OFFSET), &now, sizeof(struct rtc_time));
				((uint32_t*)mmc_data)[MMC_LAST_DATA_IDX_OFFSET] = mmc_block_num;
				/* Write to block number 2 (MMC_LAST_DATA_IDX_BLOCK) */
				ret = sdmmc_write_block(&micro_sd, MMC_LAST_DATA_IDX_BLOCK, mmc_data);
				if (ret == 0) {
					uprintf(UART0, "Wrote last bock number to block %d\n", MMC_LAST_DATA_IDX_BLOCK);
				} else {
					uprintf(UART0, "Error while writting last bock number: %d\n", ret);
				}
				memset(mmc_data, 0, MMC_BUF_SIZE);
			}
			/* Prepare buffer for next set of data */
			if (1) {
				int offset = 0;
				/* Start with magic */
				memcpy(mmc_data, MMC_BLK_VALID_MAGIC, sizeof(MMC_BLK_VALID_MAGIC));
				offset = 32;
				/* Add date */
				rtc_pcf85363_time_read(&rtc_conf, &now);
				memcpy((mmc_data + offset), &now, sizeof(struct rtc_time));
				offset += sizeof(struct rtc_time);
				/* Then add internal temp */
				memcpy((mmc_data + offset), &tmp101_deci_degrees, sizeof(tmp101_deci_degrees));
				offset += sizeof(tmp101_deci_degrees);
				/* Add battery voltage */
				memcpy((mmc_data + offset), &mv_batt, sizeof(mv_batt));
				offset += sizeof(mv_batt);
				mmc_idx = 64;
			}
			/* Display internal temp and battery level */
			if (1) {
				int abs_deci = tmp101_deci_degrees;
				if (tmp101_deci_degrees < 0) {
					abs_deci = -tmp101_deci_degrees;
				}
				uprintf(UART0, "Internal Temp : % 4d.%02d\n", (tmp101_deci_degrees / 10), (abs_deci % 10));
				uprintf(UART0, "Batterie: %d.%03d V\n", (mv_batt/1000), ((mv_batt/10)%100));
			}
			mmc_block_num++;
		}

		if (perform_time_update == 1) {
			char buff[30];
			perform_time_update = 0;
			rtc_pcf85363_time_write(&rtc_conf, &time_update);
			rtc_pcf85363_time_to_str(&time_update, buff, 30);
			uprintf(UART0, "Using updated time as now : %s\n", buff);
			time_valid = 1;
		}
	}
	return 0;
}



