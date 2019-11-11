/****************************************************************************
 *   apps/dmx/up2rf/main.c
 *
 * DMX UP2RF example
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


/* DMX informations :
 *   https://en.wikipedia.org/wiki/DMX512
 *
 * DMX uses an RS485 line with a few glitches (the break and start pulse).
 * The DMX module uses an ADM2482 isolated RS485 bridge from analog devices.
 */

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/status_led.h"
#include "drivers/adc.h"
#include "drivers/ssp.h"

#include "extdrv/max31855_thermocouple.h"


#define MODULE_VERSION    0x02
#define MODULE_NAME "DMX Module"


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
	{ LPC_UART0_RTS_PIO_0_0, LPC_IO_DIGITAL },
	/* UART 1 */
	{ LPC_UART1_RX_PIO_0_8,  LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9,  LPC_IO_DIGITAL },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};

const struct pio_config adc_pins[] = {
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0, LPC_IO_ANALOG },
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;



/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	set_pins(adc_pins);
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


/* Put any data received from RS485 UART in rs485_rx_buff
 */
void rs485_rx(uint8_t c)
{
}

void cmd_rx(uint8_t c)
{
}

/* Thermocouple reading */
const struct max31855_sensor_config thermo = {
	.ssp_bus_num = 0,
	.chip_select = LPC_GPIO_0_26,
};


const struct pio frame_tx_pin = LPC_GPIO_0_2;
const struct pio frame_en_pin = LPC_GPIO_0_0;

void dmx_send_frame(uint8_t start_code, uint8_t* slots, uint16_t nb_slots)
{
	uint16_t sent = 0;

	/* Configure pins for Start of frame transmission */
	config_gpio(&frame_tx_pin, LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 0);
	config_gpio(&frame_en_pin, LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 1);
	/* Send break : 92us minimum
	 * Pins default state chosen to generate the break condition */
	usleep(100);
	/* Send Mark-After-Break (MAB) of 12us minimum */
	gpio_set(frame_tx_pin);
	usleep(12);

	/* Configure back for RS485 */
	/* Make sure IO_Config is clocked */
	io_config_clk_on();
	LPC_IO_CONTROL->pio0_0 = (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL);
	LPC_IO_CONTROL->pio0_2 = (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL);


	/* Send start code */
	serial_send_quickbyte(UART0, start_code);

	/* And send slots data */
	while (sent < nb_slots) {
		int tmp = serial_write(UART0, (char*)(slots + sent), (nb_slots - sent));
		if (tmp == -1) {
			break;
		}
		sent += tmp;
	}

	/* Config done, power off IO_CONFIG block */
	io_config_clk_off();
}

#define DMX_NB_SLOTS 512
#define NB_VAL 20

/***************************************************************************** */
int main(void)
{
	uint16_t isnail_values[NB_VAL];
	uint8_t slots[DMX_NB_SLOTS + 1];
	uint8_t idx = 0;

	system_init();
	uart_set_config(0, (LPC_UART_8BIT | LPC_UART_NO_PAR | LPC_UART_2STOP));
	uart_on(UART0, 250000, rs485_rx); /* FIXME : configure for 250 kbits/s and 8N2 */
	uart_on(UART1, 115200, cmd_rx);
	ssp_master_on(thermo.ssp_bus_num, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);
	adc_on(NULL);

	/* RS485 config */
	if (1) {
		uint32_t rs485_ctrl = LPC_RS485_ENABLE;
		//rs485_ctrl |= LPC_RS485_DIR_PIN_RTS | LPC_RS485_AUTO_DIR_EN | LPC_RS485_DIR_CTRL_INV;
		rs485_ctrl |= LPC_RS485_DIR_PIN_RTS | LPC_RS485_AUTO_DIR_EN | LPC_RS485_DIR_CTRL_INV;
		uart_set_mode_rs485(0, rs485_ctrl, 0, 1);
	}

	/* Thermocouple configuration */
	max31855_sensor_config(&thermo);
	uprintf(UART1, "Thermocouple config OK\n");

	/* Start ADC sampling */
	adc_start_burst_conversion(ADC_MCH(1) | ADC_MCH(2), LPC_ADC_SEQ(0));

	while (1) {
		uint16_t dmx_val = 0;
		uint16_t isnail_val = 0;

		/* Send DMX frame */
		adc_get_value(&dmx_val, LPC_ADC(2));
		slots[0] = dmx_val / 4;
		if (slots[0] > 255) {
			slots[0] = 255;
		}
		dmx_send_frame(0x00, slots, 1);
		uprintf(UART1, "DMX: %d\n", slots[0]);
		msleep(100);

		adc_get_value(&isnail_val, LPC_ADC(1));
		/* Convert to mA value */
		isnail_val = ((isnail_val * 32) * 2); /* 3.2mV / digit, 50mV -> 1A */
		/* Store value */
		isnail_values[idx++] = isnail_val;
		if (idx == NB_VAL) {
			idx = 0;
		}
		/* Compute average value when we sampled enough values */
		if ((idx == 0) || (idx == (NB_VAL / 2))) {
			uint32_t moyenne = 0;
			int i = 0;
			for (i = 0; i < NB_VAL; i++) {
				moyenne += isnail_values[i];
			}
			moyenne = moyenne / NB_VAL;
			/* Display */
			uprintf(UART1, "I: %d,%04d\n", (moyenne / 1000), (moyenne % 1000));
		}

		/* Get thermocouple value */
		if (1) {
			int centi_degrees = 0, ret = 0;
			ret = max31855_sensor_read(&thermo, NULL, &centi_degrees);
			if (ret != 0) {
				uprintf(UART1, "Temp read error : %d\n", ret);
			} else {
				int abs_centi = centi_degrees;
				if (centi_degrees < 0) {
					abs_centi = -centi_degrees;
				}
				uprintf(UART1, "Temp : % 4d.%02d\n", (centi_degrees / 100), (abs_centi % 100));
			}
		}
	}
	return 0;
}



