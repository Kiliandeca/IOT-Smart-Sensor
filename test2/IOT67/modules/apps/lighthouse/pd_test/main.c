/****************************************************************************
 *   apps/lighthouse/pd_test/main.c
 *
 * Deep Power Down example
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
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/cc1101.h"
#include "drivers/ssp.h"
#include "drivers/rtc.h"
#include "extdrv/ws2812.h"

#define MODULE_VERSION  0x03
#define MODULE_NAME  "LightHouse"

#define RF_868MHz  1
#define RF_915MHz  0
#if ((RF_868MHz) + (RF_915MHz) != 1)
#error Either RF_868MHz or RF_915MHz MUST be defined.
#endif

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
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* GPIO */
	{ LPC_GPIO_0_6, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },
	{ LPC_GPIO_0_12, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },
	{ LPC_GPIO_0_15, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },
	{ LPC_GPIO_0_30, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },
	ARRAY_LAST_PIO,
};

const struct pio cc1101_cs_pin = LPC_GPIO_0_15;
const struct pio cc1101_miso_pin = LPC_SSP0_MISO_PIO_0_16;
const struct pio cc1101_gdo0 = LPC_GPIO_0_6;


const struct pio ws2812_data_out_pin = LPC_GPIO_0_30; /* Led control data pin */
const struct pio button = LPC_GPIO_0_12;


/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_brown_out_detection_config(0); /* No ADC used */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();

	rtc_on();
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


static uint8_t rf_specific_settings[] = {
	CC1101_REGS(gdo_config[2]), 0x07, /* GDO_0 - Assert on CRC OK | Disable temp sensor */
	CC1101_REGS(gdo_config[0]), 0x2E, /* GDO_2 - FIXME : do something usefull with it for tests */
	CC1101_REGS(pkt_ctrl[0]), 0x0F, /* Accept all sync, CRC err auto flush, Append, Addr check and Bcast */
};

/* RF config */
void rf_config(void)
{
	config_gpio(&cc1101_gdo0, LPC_IO_MODE_PULL_UP, GPIO_DIR_IN, 0);
	cc1101_init(0, &cc1101_cs_pin, &cc1101_miso_pin); /* ssp_num, cs_pin, miso_pin */
	/* Set default config */
	cc1101_config();
	/* And change application specific settings */
	cc1101_update_config(rf_specific_settings, sizeof(rf_specific_settings));
}

void send_on_rf(char *data, int len)
{
	uint8_t cc_tx_data[32 + 2];

	/* Create a local copy */
	memcpy((char*)&(cc_tx_data[2]), data, len);
	/* Prepare buffer for sending */
	cc_tx_data[0] = len + 1;
	cc_tx_data[1] = 0; /* Broadcast */
	/* Send */
	if (cc1101_tx_fifo_state() != 0)
		cc1101_flush_tx_fifo();
	cc1101_send_packet(cc_tx_data, (len + 2));
}

/******************************************************************************/
/* RTC events handling */
uint32_t rtc_event = 0;
void rtc_test(uint32_t tick)
{
	rtc_event = tick;
}

uint32_t isp_btn_request = 0;
void isp_btn(uint32_t gpio)
{
	isp_btn_request = 1;
}

uint32_t wake_btn_request = 0;
void wake_test(uint32_t gpio)
{
	wake_btn_request = 1;
}

/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, NULL);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */
	set_rtc_callback(&rtc_test, 0, 25);
	set_gpio_callback(&isp_btn, &button, EDGE_RISING);

	/* Led strip configuration */
	ws2812_config(&ws2812_data_out_pin);
	ws2812_clear();

	/* RF Config */
	rf_config();
	uprintf(UART0, "Config done.\n");

	while (1) { /* Note that in this example we do not loop even once */
		int i = 0;

		if (wake_btn_request == 1) {
			uprintf(UART0, "Button Wake request\n");
			wake_btn_request = 0;
		}

		for (i = 0; i < 5; i++) {
			ws2812_set_pixel(0, (50 - (i * 10)), 0, (i * 10));
			ws2812_send_frame(0);
			if (rtc_event != 0) {
				uprintf(UART0, "RTC: %d\n", rtc_event);
				rtc_event = 0;
			}
			if (isp_btn_request == 1) {
				uprintf(UART0, "ISP Button\n");
				isp_btn_request = 0;
			}
			msleep(1000);
		}

		cc1101_power_down();
		uprintf(UART0, "RF down\n");
		serial_flush(UART0);
		ws2812_clear();
		msleep(50);
		enter_deep_power_down();
		uprintf(UART0, "Wake-up ...\n"); /* Never displayed, this is deep power down ! */
	}

	return 0;
}

