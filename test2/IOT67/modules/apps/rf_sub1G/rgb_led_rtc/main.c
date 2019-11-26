/****************************************************************************
 *   apps/rf_sub1G/rgb_led_rtc/main.c
 *
 * sub1G_module support code - USB version
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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
#include "drivers/i2c.h"
#include "drivers/adc.h"
#include "drivers/timers.h"
#include "drivers/rtc.h"
#include "extdrv/cc1101.h"
#include "extdrv/status_led.h"
#include "extdrv/tmp101_temp_sensor.h"


#define MODULE_VERSION	0x02
#define MODULE_NAME "RF Sub1G - USB"


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
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* ADC */
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	/* Timer 3 */
#define LPC_TIMER_PIN_CONFIG   (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL | LPC_IO_DRIVE_HIGHCURENT)
	{ LPC_TIMER_32B1_M0_PIO_0_23, LPC_TIMER_PIN_CONFIG }, /* RGB Led Red */
	{ LPC_TIMER_32B1_M1_PIO_0_24, LPC_TIMER_PIN_CONFIG }, /* RGB Led Green */
	{ LPC_TIMER_32B1_M2_PIO_0_25, LPC_TIMER_PIN_CONFIG }, /* RGB Led Blue */
	ARRAY_LAST_PIO,
};

const struct pio cc1101_cs_pin = LPC_GPIO_0_15;
const struct pio cc1101_miso_pin = LPC_SSP0_MISO_PIO_0_16;
const struct pio cc1101_gdo0 = LPC_GPIO_0_6;
const struct pio cc1101_gdo2 = LPC_GPIO_0_7;

#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};
const struct pio temp_alert = LPC_GPIO_0_3;

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;

static uint16_t leds[3] = { 0, 0, 0 };

#define ADC_VBAT  LPC_ADC(0)
#define ADC_EXT1  LPC_ADC(1)
#define ADC_EXT2  LPC_ADC(2)

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
	/* RTC */
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


/***************************************************************************** */
void rgb_config(void)
{
	/* Timer configuration */
	struct lpc_timer_pwm_config timer_conf = {
		.nb_channels = 3,
		.period_chan = 3,
		.period = 180,
		.outputs = { 0, 1, 2, },
		.match_values = { 20, 125, 180, },
	};
	timer_pwm_config(LPC_TIMER_32B1, &timer_conf);

	/* Start the timer */
	timer_start(LPC_TIMER_32B1);
}

/***************************************************************************** */
/* Temperature */
/* The Temperature Alert pin is on GPIO Port 0, pin 7 (PIO0_7) */
/* The I2C Temperature sensor is at address 0x94 */
void WAKEUP_Handler(void)
{
}

void temp_config()
{
	int ret = 0;

	/* Temp Alert */
	config_gpio(&temp_alert, LPC_IO_MODE_PULL_UP, GPIO_DIR_IN, 0);
	/* FIXME : add a callback on temp_alert edge */

	/* Temp sensor */
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(UART0, "Temp config error: %d\n", ret);
	}
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
	if (1) {
		uprintf(UART0, "CC1101 RF link init done.\n");
	}
#endif
}

void send_uint16_on_RF(uint8_t addr, uint8_t type, uint16_t val)
{
	uint8_t cc_tx_data[RF_BUFF_LEN + 2];
	cc_tx_data[0] = 4;
	cc_tx_data[1] = addr;
	cc_tx_data[2] = type;
	cc_tx_data[3] = ((val >> 8) & 0xff);
	cc_tx_data[4] = (val & 0xff);
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}
	cc1101_send_packet(cc_tx_data, 5);
}
void send_error_on_rf(uint8_t addr, uint8_t err_code)
{
	uint8_t cc_tx_data[RF_BUFF_LEN + 2];
	cc_tx_data[0] = 3;
	cc_tx_data[1] = addr;
	cc_tx_data[2] = 'E';
	cc_tx_data[3] = err_code;
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
	if (1) {
		uprintf(UART0, "RF: ret:%d, st: %d.\n", ret, status);
	}
#endif

	switch (data[2]) {
		case 'T':
			{
				uint16_t val = 0;
				int deci_degrees = 0;
				/* Read the temperature */
				ret = tmp101_sensor_read(&tmp101_sensor, &val, &deci_degrees);
				if (ret != 0) {
					send_uint16_on_RF(RF_BROADCAST, 'T', val);
				} else {
					send_error_on_rf(RF_BROADCAST, 'T');
				}
#ifdef DEBUG
				if (ret != 0) {
					uprintf(UART0, "Temp read error: %d\n", ret);
				} else {
					uprintf(UART0, "Temp read: %d,%d - raw: 0x%04x.\n",
							(deci_degrees/10), (deci_degrees%10), val);
				}
#endif
			}
			break;
		case 'V':
			{
				uint16_t val = 0;
				/* Get and display the battery voltage */
				ret = adc_get_value(&val, ADC_VBAT);
				if (ret != 0) {
					send_uint16_on_RF(RF_BROADCAST, 'V', val);
				} else {
					send_error_on_rf(RF_BROADCAST, 'V');
				}
#ifdef DEBUG
				if (ret >= 0) {
					int milli_volts = ((val * 32) / 10);
					uprintf(UART0, "Vbat: %d (raw: 0x%04x)\n", (milli_volts * 2), val);
				}
#endif
			}
		default: /* This is a color code, three ascii values */
			{
				leds[0] = ((data[2] - '0') * 20);
				timer_set_match(LPC_TIMER_32B1, 0, leds[0]);
				leds[1] = ((data[3] - '0') * 20);
				timer_set_match(LPC_TIMER_32B1, 1, leds[1]);
				leds[2] = ((data[4] - '0') * 20);
				timer_set_match(LPC_TIMER_32B1, 2, leds[2]);
			}
			break;
	}
}


/* Data sent on radio comes from the UART, put any data received from UART in
 * cc_tx_buff and send when either '\r' or '\n' is received.
 * This function is very simple and data received between cc_tx flag set and
 * cc_ptr rewind to 0 may be lost. */
static volatile uint32_t cc_tx = 0;
static volatile uint8_t cc_tx_buff[RF_BUFF_LEN];
static volatile uint8_t cc_ptr = 0;
void handle_uart_cmd(uint8_t c)
{
	if (cc_ptr < RF_BUFF_LEN) {
		cc_tx_buff[cc_ptr++] = c;
	} else {
		cc_ptr = 0;
	}
	if ((c == '\n') || (c == '\r')) {
		cc_tx = 1;
	}
}

void send_uart_to_rf(void)
{
	uint8_t cc_tx_data[RF_BUFF_LEN + 2];
	uint8_t tx_len = cc_ptr;
	int ret = 0;

	/* Create a local copy */
	memcpy((char*)&(cc_tx_data[2]), (char*)cc_tx_buff, tx_len);
	/* "Free" the rx buffer as soon as possible */
	cc_ptr = 0;
	/* Prepare buffer for sending */
	cc_tx_data[0] = tx_len + 1;
	cc_tx_data[1] = RF_BROADCAST; /* Broadcast */
	/* Send */
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}
	ret = cc1101_send_packet(cc_tx_data, (tx_len + 2));

#ifdef DEBUG
	{
		/* Give some feedback on UART 0 */
		uprintf(UART0, "Tx ret: %d\n", ret);
	}
#endif
}


/***************************************************************************** */
static volatile uint32_t rtc_tick = 0;
void rtc_toggle_led(uint32_t date)
{
	static int led_num = 0;

	rtc_tick = date;
	/* Restore led value */
	timer_set_match(LPC_TIMER_32B1, led_num, leds[led_num]);
	/* Turn off the next one */
	led_num = ((led_num + 1) % 3);
	timer_set_match(LPC_TIMER_32B1, led_num, (9 * 20));
	/* Set a new value  to the third */
	led_num = ((led_num + 1) % 3);
	timer_set_match(LPC_TIMER_32B1, led_num, ((date % 18) * 10));
}


/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, handle_uart_cmd);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	adc_on(NULL);
	status_led_config(&status_led_green, &status_led_red);
	timer_on(LPC_TIMER_32B1, 0, NULL);
	set_rtc_callback(&rtc_toggle_led, 0, 1);

	/* Radio */
	rf_config();

	/* Temperature sensor */
	temp_config();

	/* RGB Led */
	rgb_config();

	while (1) {
		uint8_t status = 0;
		/* Request a Temp conversion on I2C TMP101 temperature sensor */
		tmp101_sensor_start_conversion(&tmp101_sensor); /* A conversion takes about 40ms */
		/* Start an ADC conversion to get battery voltage */
		adc_start_convertion_once(ADC_VBAT, LPC_ADC_SEQ(0), 0);

		status_led(none);
		/* RF */
		if (cc_tx == 1) {
			send_uart_to_rf();
			cc_tx = 0;
		}
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
#ifdef DEBUG
			if (1) {
				/* Give some feedback on UART 0 */
				struct lpc_rtc* rtc = LPC_RTC;
				uprintf(UART0, "RTC last tick: %d, match: %d\r\n", rtc_get_count(), rtc->match);
		}
#endif
		}
#ifdef DEBUG
		if (rtc_tick != 0) {
			/* Give some feedback on UART 0 */
			uprintf(UART0, "RTC last tick: %d\r\n", rtc_tick);
			rtc_tick = 0;
		}
#endif
	}
	return 0;
}




