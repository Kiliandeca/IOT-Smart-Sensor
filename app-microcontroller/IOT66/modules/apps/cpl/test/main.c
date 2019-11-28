/****************************************************************************
 *   apps/cpl/test/main.c
 *
 * Copyright 2015-2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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
#include "extdrv/status_led.h"
#include "drivers/adc.h"
#include "drivers/ssp.h"

#include "extdrv/max31855_thermocouple.h"


#define MODULE_VERSION    0x02
#define MODULE_NAME "CPL Module"


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

const struct pio cpl_detect = LPC_GPIO_0_6;
const struct pio cpl_select = LPC_GPIO_0_7;


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


void uart_cmd_rx(uint8_t c)
{
}

/* Thermocouple reading */
const struct max31855_sensor_config thermo = {
	.ssp_bus_num = 0,
	.chip_select = LPC_GPIO_0_26,
};

#define STR_MAX 40


/* CPL config defaults in UART mode :
 * Mains signal freq : 132.5kHz
 * Baud Rate : 2400 bauds/s
 * Deviation : 0.5
 * Watchdog enabled
 * Transmission time out : 1sec.
 * Frequency detection time : 1ms
 * Detection method : Carrier detection without conditioning :
 *   Carrier detection notification on CD_PD Line CLR/T and RxD signal always present.
 * Mains Interfacing Mode : Asynchronous
 * Output clock : 4MHz
 * Output Voltage Level Freeze : disabled
 * Header Recognition : disabled
 * Frame Length Count : disabled
 * Header Length : 16 bits
 * Extended registers disabled (Control Register Extended Mode disabled)
 * Normal sensitivity mode
 * Input filter disabled
 * Frame header : 0x9B58
 * Frame length (number of 16 bits words expected) : 8 half words
 */

uint32_t cpl_sof_detected = 0;
void cpl_get_frame(uint32_t gpio)
{
	cpl_sof_detected = 1;
}

void cpl_rx(uint8_t c)
{
	serial_send_quickbyte(0, c);
}

void cpl_send_frame(uint8_t* data, uint16_t len)
{
	uint16_t sent = 0;

	/* Enter Tx mode */
	gpio_clear(cpl_select);
	/* Wait at least Tcc time (See ST7540_power_Line_modem deatasheet pages 20 and table 5 page 14 */
	/* Tcc is two bit times, which is 2 * 417 us at 2400 bauds/s : 834 us */
	/* Tests shows that 8 bit times is better */
	msleep(4);

	/* Send data */
	while (sent < len) {
		int tmp = serial_write(1, (char*)(data + sent), (len - sent));
		if (tmp == -1) {
			break;
		}
		sent += tmp;
	}
	serial_flush(1);

	/* Back to Rx mode */
	msleep(16); /* Wait at least 20 bit times (2 bytes) */
	gpio_set(cpl_select);
}

#define NB_VAL 20

/***************************************************************************** */
int main(void)
{
	uint16_t isnail_values[NB_VAL];
	uint8_t idx = 0;

	system_init();
	uart_on(UART0, 115200, uart_cmd_rx);
	uart_on(UART1, 2400, cpl_rx);
	ssp_master_on(thermo.ssp_bus_num, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);
	adc_on(NULL);

	/* Configure pins for CPL control signals */
	config_gpio(&cpl_select, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL), GPIO_DIR_OUT, 1); /* 1 is Rx mode */
	set_gpio_callback(cpl_get_frame, &cpl_detect, EDGE_FALLING);

	/* Thermocouple configuration */
	max31855_sensor_config(&thermo);
	uprintf(UART0, "Thermocouple config OK\n");

	/* Start ADC sampling */
	adc_start_burst_conversion(ADC_MCH(1) | ADC_MCH(2), LPC_ADC_SEQ(0));

	while (1) {
		uint16_t acs_val = 0, isnail_val = 0;

		adc_get_value(&acs_val, LPC_ADC(1)); /* ADC current consumption value */

		adc_get_value(&isnail_val, LPC_ADC(2));
		/* Convert to mA value */
		isnail_val = ((isnail_val * 32) * 2); /* 3.2mV / digit, 50mV -> 1A */
		/* Store value */
		isnail_values[idx++] = isnail_val;
		if (idx == NB_VAL) {
			idx = 0;
		}
		if (cpl_sof_detected == 1) {
			cpl_sof_detected = 0;
			uprintf(UART0, "SOF detected\n");
		}

		/* Get thermocouple value */
		if (idx == 0) {
			int centi_degrees = 0, ret = 0;
			ret = max31855_sensor_read(&thermo, NULL, &centi_degrees);
			if (ret != 0) {
				uprintf(UART0, "Temp read error : %d\n", ret);
			} else {
				char moy_str[STR_MAX];
				uint8_t len = 0;
				int abs_centi = centi_degrees;
				uint32_t moyenne = 0;
				int i = 0;
				if (centi_degrees < 0) {
					abs_centi = -centi_degrees;
				}
				uprintf(UART0, "Temp : % 4d.%02d\n", (centi_degrees / 100), (abs_centi % 100));
				/* And send over CPL */
				len = snprintf(moy_str, STR_MAX, "T: %d,%02d\n", (centi_degrees / 100), (centi_degrees % 100));
				cpl_send_frame((uint8_t*)moy_str, len);
				msleep(250);
				/* Compute average value as we sampled enough values */
				for (i = 0; i < NB_VAL; i++) {
					moyenne += isnail_values[i];
				}
				moyenne = moyenne / NB_VAL;
				/* Display */
				uprintf(UART0, "I: %d,%04d\n", (moyenne / 1000), (moyenne % 1000));
				/* And send over CPL */
				len = snprintf(moy_str, STR_MAX, "I: %d,%04d\n", (moyenne / 1000), (moyenne % 1000));
				cpl_send_frame((uint8_t*)moy_str, len);
				msleep(250);
			}
		}

		msleep(25);
	}
	return 0;
}



