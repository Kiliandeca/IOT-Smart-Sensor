/****************************************************************************
 *   apps/base/usd_card/main.c
 *
 * SD/MMC card example
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


/*
 * This example reads data from the first 16 bytes of the first four bolcs of
 * the SD card to collect a message to be displayed, and then stores temperature
 * data on the SD, with one sample on each successive block, starting on the
 * fifth block.
 *
 * See README for more info.
 */


#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "lib/errno.h"
#include "lib/utils.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"
#include "drivers/i2c.h"

#include "extdrv/status_led.h"
#include "extdrv/tmp101_temp_sensor.h"
#include "extdrv/sdmmc.h"

#define MODULE_VERSION    0x04
#define MODULE_NAME "GPIO Demo Module"


#define SELECTED_FREQ  FREQ_SEL_48MHz


/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 : Config / Debug / USB */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* I2C : TMP101 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* SPI : uSD card */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* GPIO */
	{ LPC_GPIO_1_6, LPC_IO_DIGITAL },  /* uSD Card SPI Chip Select */
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;


/***************************************************************************** */
/* TMP101 onboard I2C temperature sensor */
#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};

void temp_config(int uart_num)
{
	int ret = 0;
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(uart_num, "Temp config error: %d\n", ret);
	}
}



/***************************************************************************** */
/* SD/MMC Card */
struct sdmmc_card micro_sd = {
	.ssp_bus_num = SSP_BUS_0,
	.card_type = MMC_CARDTYPE_UNKNOWN,
	.block_size = 16,
	.chip_select = LPC_GPIO_0_15,
};
#define MMC_BUF_SIZE   64
uint8_t mmc_data[MMC_BUF_SIZE];


/***************************************************************************** */
void system_init()
{
	/* Configure the Watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_brown_out_detection_config(0); /* No ADC used */
	system_set_default_power_state();
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



/***************************************************************************** */
int main(void)
{
	int ret = 0;
	system_init();
	uart_on(UART0, 115200, NULL);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);

	/* TMP101 sensor config */
	temp_config(UART0);

	/* microSD card init */
	do {
		ret = sdmmc_init(&micro_sd);
		if (ret == 0) {
			msleep(10);
			ret = sdmmc_init_wait_card_ready(&micro_sd);
			if (ret == 0) {
				ret = sdmmc_init_end(&micro_sd);
			}
		}
		uprintf(UART0, "uSD init: %d, type: %d, bs: %d\n", ret, micro_sd.card_type, micro_sd.block_size);
	} while (ret != 0);
	memset(mmc_data, 0, MMC_BUF_SIZE);
	ret = sdmmc_read_block(&micro_sd, 0, mmc_data);
	ret = sdmmc_read_block(&micro_sd, 1, (mmc_data + 16));
	ret = sdmmc_read_block(&micro_sd, 2, (mmc_data + 32));
	ret = sdmmc_read_block(&micro_sd, 3, (mmc_data + 48));
	uprintf(UART0, "uSD read: %s\n", mmc_data);

	msleep(50);

	while (1) {
		int tmp101_deci_degrees = 0;
		int abs_deci = 0;
		static int block_num = 4;

		/* Do not write too many times, one data every 20s is more than enough for a test */
		msleep(20 * 1000);

		/* Get internal temperature */
		tmp101_sensor_start_conversion(&tmp101_sensor);
		msleep(40);
		ret = tmp101_sensor_read(&tmp101_sensor, NULL, &tmp101_deci_degrees);
		if (ret != 0) {
			uprintf(UART0, "TMP101 read error : %d\n", ret);
		}
		if (tmp101_deci_degrees < 0) {
			abs_deci = -tmp101_deci_degrees;
		} else {
			abs_deci = tmp101_deci_degrees;
		}

		/* Write time and temp to uSD */
		memset(mmc_data, 0, MMC_BUF_SIZE);
		mmc_data[0] = tmp101_deci_degrees / 10;
		mmc_data[1] = abs_deci % 10;
		ret = sdmmc_write_block(&micro_sd, block_num++, mmc_data);
		uprintf(UART0, "Wrote data on block %d: ret=%d\n", (block_num - 1), ret);

		/* Display */
		uprintf(UART0, "Internal Temp : % 4d.%02d\n", (tmp101_deci_degrees / 10), (abs_deci % 10));
	}
	return 0;
}


