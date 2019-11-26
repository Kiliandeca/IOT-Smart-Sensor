/****************************************************************************
 *   apps/cape_rccar/remote//main.c
 *
 * Remote controller using sub1G_module
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
#include "drivers/ssp.h"
#include "drivers/adc.h"
#include "drivers/timers.h"
#include "extdrv/cc1101.h"
#include "extdrv/status_led.h"


#define MODULE_VERSION	0x02
#define MODULE_NAME "RF Sub1G"


#define RF_868MHz  1
#define RF_915MHz  0
#if ((RF_868MHz) + (RF_915MHz) != 1)
#error Either RF_868MHz or RF_915MHz MUST be defined.
#endif


#define DEBUG 1
#define BUFF_LEN 60

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
	/* ADC */
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	ARRAY_LAST_PIO,
};

const struct pio cc1101_cs_pin = LPC_GPIO_0_15;
const struct pio cc1101_miso_pin = LPC_SSP0_MISO_PIO_0_16;
const struct pio cc1101_gdo0 = LPC_GPIO_0_6;
const struct pio cc1101_gdo2 = LPC_GPIO_0_7;

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;


#define ADC_VBAT  LPC_ADC(0)
#define ADC_EXT1  LPC_ADC(1)
#define ADC_EXT2  LPC_ADC(2)

const struct wdt_config wdconf = {
    .clk_sel = WDT_CLK_IRC,
    .intr_mode_only = 0,
    .callback = NULL,
    .locks = 0,
    .nb_clk = 0x04FFFF,
    .wdt_window = 0,
    .wdt_warn = 0x3FF,
};


/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
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



/******************************************************************************/
/* RF Communication */
#define RF_BUFF_LEN  64
#define RF_BROADCAST 0

static volatile int check_rx = 0;
void rf_rx_calback(uint32_t gpio)
{
	check_rx = 1;
}

static uint8_t rf_specific_settings[] = {
	CC1101_REGS(gdo_config[2]), 0x07, /* GDO_0 - Assert on CRC OK | Disable temp sensor */
	CC1101_REGS(gdo_config[0]), 0x2E, /* GDO_2 - FIXME : do something usefull with it for tests */
	CC1101_REGS(pkt_ctrl[0]), 0x0F, /* Accept all sync, CRC err auto flush, Append, Addr check and Bcast */
	CC1101_REGS(radio_stm[1]), 0x3F, /* CCA mode "if RSSI below threshold", Stay in RX, Go to RX (page 81) */
	CC1101_REGS(agc_ctrl[1]), 0x20, /* LNA 2 gain decr first, Carrier sense relative threshold set to 10dB increase in RSSI value */
#if (RF_915MHz == 1)
	/* FIXME : Add here a define protected list of settings for 915MHz configuration */
#endif
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
	set_gpio_callback(rf_rx_calback, &cc1101_gdo0, EDGE_RISING);

#ifdef DEBUG
	uprintf(UART0, "CC1101 RF link init done.\n");
#endif
}

void send_on_RF(uint8_t addr, uint8_t type, uint8_t val)
{
	uint8_t cc_tx_data[8];
	cc_tx_data[0] = 3;
	cc_tx_data[1] = addr;
	cc_tx_data[2] = type;
	cc_tx_data[3] = val;
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}
	cc1101_send_packet(cc_tx_data, 4);
}

void handle_rf_rx_data(void)
{
	uint8_t data[RF_BUFF_LEN];
	int8_t ret = 0;
	uint8_t status = 0;

	/* Check for received packet (and get it if any) */
	ret = cc1101_receive_packet(data, RF_BUFF_LEN, &status);
	/* Go back to RX mode */
	cc1101_enter_rx_mode();

#ifdef DEBUG
	uprintf(UART0, "RF: ret:%d, st: %d.\n", ret, status);
#endif

}


/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, NULL);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */
	adc_on(NULL);
	status_led_config(&status_led_green, &status_led_red);

	/* Radio */
	rf_config();

	/*	ADC */
	adc_start_burst_conversion(ADC_MCH(ADC_EXT1) | ADC_MCH(ADC_EXT2), LPC_ADC_SEQ(0));

	/* Config done, start watchdog */
	watchdog_config(&wdconf);

	while (1) {
		uint8_t status = 0;
		uint16_t val = 0;
		uint8_t dir = 0, speed = 0;

		status_led(none);
		watchdog_feed();
		/* Get current ADC values and compute new commands */
		/* First axis : Direction */
		adc_get_value(&val, ADC_EXT1);
		dir = (val / 17) + 59; /* This gives a 90 as middle position, with a value between 60 and 120 */
		send_on_RF(0, 'D', dir);
#if 0
		uprintf(UART0, "Direction : %d\n", dir);
#endif
		status_led(green_on);
		msleep(10);

		adc_get_value(&val, ADC_EXT2);
		speed = ((1024 - val) / 12) + 50; /* About same result, inverting axis to match joystick output */
		send_on_RF(0, 'S', speed);
#if 0
		uprintf(UART0, "Speed : %d\n", speed);
#endif
		status_led(red_on);
		msleep(10);

		/* RF */
		/* Do not leave radio in an unknown or unwated state */
		do {
			status = (cc1101_read_status() & CC1101_STATE_MASK);
		} while (status == CC1101_STATE_TX);

		if (status != CC1101_STATE_RX) {
			static uint8_t loop = 0;
			loop++;
			if (loop > 10) {
				if (cc1101_rx_fifo_state() != 0) {
					cc1101_flush_rx_fifo();
				}
				cc1101_enter_rx_mode();
				loop = 0;
			}
		}
		if (check_rx == 1) {
			check_rx = 0;
			handle_rf_rx_data();
		}
	}
	return 0;
}




