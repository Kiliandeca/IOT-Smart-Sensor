/****************************************************************************
 *   apps/rf_sub1G/temp_sensor/main.c
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
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"
#include "drivers/i2c.h"
#include "drivers/adc.h"
#include "drivers/timers.h"
#include "extdrv/cc1101.h"
#include "extdrv/status_led.h"
#include "extdrv/tmp101_temp_sensor.h"
#include "lib/stdio.h"
#include "lib/protocols/dtplug/slave.h"


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

const struct pio temp_alert = LPC_GPIO_0_3;

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;


static struct dtplug_protocol_handle uart_handle;


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

#define TMP101_ADDR_00  0x90 /* Pin Addr0 (pin5 of tmp101) connected to GND */
#define TMP101_ADDR_01  0x94 /* Pin Addr0 connected to VCC */
struct tmp101_sensor_config tmp101_sensor_0 = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR_00,
	.resolution = TMP_RES_ELEVEN_BITS,
};
struct tmp101_sensor_config tmp101_sensor_1 = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR_01,
	.resolution = TMP_RES_ELEVEN_BITS,
};
void temp_config()
{
	int ret = 0;

	/* Temp Alert */
	config_gpio(&temp_alert, LPC_IO_MODE_PULL_UP, GPIO_DIR_IN, 0);
	/* FIXME : add a callback on temp_alert edge */

	/* Temp sensor */
	ret = tmp101_sensor_config(&tmp101_sensor_0);
	if (ret != 0) {
		uprintf(UART0, "Temp config error on sensor 0 (%d)\n", tmp101_sensor_0.addr);
	}
	ret = tmp101_sensor_config(&tmp101_sensor_1);
	if (ret != 0) {
		uprintf(UART0, "Temp config error on sensor 1 (%d)\n", tmp101_sensor_1.addr);
	}
}



/* Data sent on radio comes from the UART when appropriate command is received. */
void handle_uart_commands(struct packet* command)
{
	uint8_t channel = 0;

	/* Many commands use only this one, get it right now. */
	if (command->info.seq_num & QUICK_DATA_PACKET) {
		channel = command->info.quick_data[0];
	} else {
		channel = command->data[0];
	}
	switch (command->info.type) {
		case PKT_TYPE_START_TEMP_CONVERSION:
			/* Request a Temp conversion on I2C TMP101 temperature sensors (A conversion takes about 40ms) */
			tmp101_sensor_start_conversion(&tmp101_sensor_0);
			tmp101_sensor_start_conversion(&tmp101_sensor_1);
			/* Get value from SPI connected thermocouple */
			break;
		case PKT_TYPE_GET_TEMPERATURE:
			{
				uint16_t values[4] = {0, 0, 0, 0};
				int i = 0;
				tmp101_sensor_read(&tmp101_sensor_0, NULL, &i);
				values[1] = (int16_t)i;
				tmp101_sensor_read(&tmp101_sensor_1, NULL, &i);
				values[0] = (int16_t)i;
				/* Swap endianness */
				for (i = 0; i < 4; i++) {
					values[i] = (uint16_t)byte_swap_16(values[i]);
				}
				/* And send reply */
				dtplug_protocol_send_reply(&uart_handle, command, NO_ERROR, 4, (uint8_t*)values);
			}
			break;
		case PKT_TYPE_START_ADC_CONVERSION:
			if (channel <= ADC_EXT2) {
				/* Start an ADC conversion on selected ADC channel */
				adc_start_convertion_once(channel, LPC_ADC_SEQ(0), 0);
			} else {
				dtplug_protocol_send_reply(&uart_handle, command, ERROR_IN_DATA_VALUES, 0, NULL);
			}
			break;
		case PKT_TYPE_GET_ADC_VALUE:
			{
				uint16_t value = 0;
				adc_get_value(&value, channel);
				dtplug_protocol_send_reply(&uart_handle, command, NO_ERROR, 2, (uint8_t*)(&value));
			}
			break;
		default:
			uprintf(UART0, "Unknown packet type : packet not handled\n");
			dtplug_protocol_send_reply(&uart_handle, command, ERROR_PKT_NOT_HANDLED, 0, NULL);
			break;
	}
	dtplug_protocol_release_old_packet(&uart_handle);
}


/***************************************************************************** */
int main(void)
{
	system_init();
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	adc_on(NULL);
	status_led_config(&status_led_green, &status_led_red);
	timer_on(LPC_TIMER_32B1, 0, NULL);
	dtplug_protocol_set_dtplug_comm_uart(0, &uart_handle);

	/* Temperature sensor */
	temp_config();

	/* RGB Led */
	rgb_config();

	while (1) {
		struct packet* pkt = NULL;

		status_led(red_off);

		/* UART */
		pkt = dtplug_protocol_get_next_packet_ok(&uart_handle);
		if (pkt != NULL) {
			handle_uart_commands(pkt);
			status_led(green_off);
		}
	}
	return 0;
}




