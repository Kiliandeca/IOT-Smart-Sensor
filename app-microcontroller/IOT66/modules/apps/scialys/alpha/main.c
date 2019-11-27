/****************************************************************************
 *   apps/scialys/alpha/main.c
 *
 * DMX module for solar-panel power generation tracking and fair use.
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


/* DMX informations :
 *   https://en.wikipedia.org/wiki/DMX512
 *
 * DMX uses an RS485 line with a few glitches (the break and start pulse).
 * The DMX module uses an ADM2482 isolated RS485 bridge from analog devices.
 */

#include "core/system.h"
#include "core/pio.h"
#include "core/systick.h"
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

/* Period of the decrementer handler from the systick interrupt */
#define DEC_PERIOD  100

/* If temperature falls bellow FORCE_HEATER_TEMP value, we enter forced heater mode, until
 *    TARGET_FORCED_HEATER_TEMP is reached.
 * When in forced heater mode, the heated is controlled to heat at FORCED_MODE_VALUE which 
 *    is between 0 and 255.
 */
#define FORCE_HEATER_TEMP  40
#define TARGET_FORCED_HEATER_TEMP 45
#define NO_FORCED_HEATING_ON_SUNNY_DAYS 750
#define FORCED_MODE_VALUE  190 /* A fraction of 255 */
uint32_t forced_heater_mode = 0;
uint32_t forced_heater_delay = 0;
uint32_t forced_heater_time = 0;

#define FORCED_HEATER_DELAY      (7 * 3600 * 1000 / DEC_PERIOD)  /* Delay before automatic forced heating */
#define FORCED_HEATER_DURATION   (3 * 3600 * 1000 / DEC_PERIOD)  /* Duration of automatic forced heating */

#define MANUAL_ACTIVATION_DURATION   (3600 * 1000 / DEC_PERIOD)  /* One hour */

uint32_t never_force = 0;

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
	/* GPIO */
	{ LPC_GPIO_0_12, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_25, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_26, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_28, LPC_IO_DIGITAL },
	{ LPC_GPIO_0_29, LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};

const struct pio_config adc_pins[] = {
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },  /* ADC1 */
	{ LPC_ADC_AD2_PIO_1_0, LPC_IO_ANALOG },   /* ADC2 */
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;

const struct pio button = LPC_GPIO_0_12; /* ISP button */
const struct pio ejp_in_pin = LPC_GPIO_0_25; 
#define DAY_IS_EJP  0  /* Input is pulled low when EJP is ON */
int ejp_in = 0;

/* Thermocouple reading */
const struct max31855_sensor_config thermo = {
	.ssp_bus_num = 0,
	.chip_select = LPC_GPIO_0_26,
};

/* DMX signals */
/* DMX signals are on LPC_GPIO_0_0 for Frame enable pin (RS485 enable) and LPC_GPIO_0_2 for
 * Frame Tx pin (RS485 Tx).
 */
const struct pio frame_tx_pin = LPC_GPIO_0_2;
const struct pio frame_en_pin = LPC_GPIO_0_0;



/***************************************************************************** */
static volatile int got_wdt_int = 0;
void wdt_callback(void)
{
	got_wdt_int = 1;
}

const struct wdt_config wdconf = {
	.clk_sel = WDT_CLK_IRC,
	.intr_mode_only = 0,
	.callback = wdt_callback,
	.locks = 0,
	.nb_clk = 0x0FFFFFF, /* 0x3FF to 0x03FFFFFF */
	.wdt_window = 0,
	.wdt_warn = 0x3FF,
};

void system_init()
{
	/* Configure the Watchdog */
	watchdog_config(&wdconf);
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

	/* Configure back for RS485.
	 * This is done using direct access to the registers as this step is time critical
	 */
	/* Make sure IO_Config is clocked */
	io_config_clk_on();
	LPC_IO_CONTROL->pio0_0 = (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL);
	LPC_IO_CONTROL->pio0_2 = (LPC_IO_FUNC_ALT(2) | LPC_IO_DIGITAL);


	/* Send start code */
	serial_send_quickbyte(0, start_code);

	/* And send slots data */
	while (sent < nb_slots) {
		int tmp = serial_write(0, (char*)(slots + sent), (nb_slots - sent));
		if (tmp == -1) {
			break;
		}
		sent += tmp;
	}

	/* Config done, power off IO_CONFIG block */
	io_config_clk_off();
}


uint32_t manual_activation_request = 0;
void manual_activation(uint32_t gpio) {
    manual_activation_request = MANUAL_ACTIVATION_DURATION;
}
void handle_dec_request(uint32_t curent_tick) {
	if (manual_activation_request > 0) {
		manual_activation_request--;
	}
	if (forced_heater_mode == 1) {
		if (forced_heater_delay > 0) {
			forced_heater_delay--;
		}
		if (forced_heater_time > 0) {
			forced_heater_time--;
		}
	}
}


/* DMX */
#define DMX_NB_SLOTS 4
#define NB_VAL 20


enum modes {
	heat = 'C',
	ejp = 'E',
	no_heat_prod = 'P',
	forced = 'F',
	temp_OK = 'T',
	manual = 'M',
	idle_heat = 'L',
	full_heat = 'F',
};

/***************************************************************************** */
int main(void)
{
	uint16_t isnail_solar_values[NB_VAL];
	uint16_t isnail_home_values[NB_VAL];
	uint8_t slots[DMX_NB_SLOTS + 1];
	uint8_t idx = 0;
	uint32_t loop = 0;
	char mode = heat; /* Debug info */

	system_init();
	status_led(red_only);
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

	/* Activate on Rising edge (button release) */
	set_gpio_callback(manual_activation, &button, EDGE_RISING);

	/* Start ADC sampling */
	adc_start_burst_conversion(ADC_MCH(1) | ADC_MCH(2), LPC_ADC_SEQ(0));

	/* Configure Input GPIO for EJP mode detection */
	config_gpio(&ejp_in_pin, 0, GPIO_DIR_IN, 0); 

	status_led(green_only);
	memset(slots, 0, (DMX_NB_SLOTS + 1));

	msleep(50);
	/* Read parameters from memory */
	if (1) {
		never_force = 0;
		forced_heater_delay = FORCED_HEATER_DELAY;
		forced_heater_time = FORCED_HEATER_DURATION;
	}

	while (1) {
		static uint8_t dmx_val = 0;
		uint32_t moyenne_solar = 0;
		uint32_t moyenne_home = 0;
		uint16_t isnail_val_solar = 0;
		uint16_t isnail_val_home = 0;
		int centi_degrees = 0;

		mode = heat;
		/* Always track power consumption and production */
		adc_get_value(&isnail_val_solar, LPC_ADC(1));
		adc_get_value(&isnail_val_home, LPC_ADC(2));
		/* Convert to mA value */
		isnail_val_solar = ((isnail_val_solar * 32) * 2); /* 3.2mV / digit, 50mV -> 1A */
		isnail_val_home = ((isnail_val_home * 32) * 2); /* 3.2mV / digit, 50mV -> 1A */
		/* Store value */
		isnail_solar_values[idx] = isnail_val_solar;
		isnail_home_values[idx++] = isnail_val_home;
		if (idx == NB_VAL) {
			idx = 0;
		}
		/* Compute average value when we sampled enough values */
		if ((idx == 0) || (idx == (NB_VAL / 2))) {
			int i = 0;
			for (i = 0; i < NB_VAL; i++) {
				moyenne_solar += isnail_solar_values[i];
				moyenne_home += isnail_home_values[i];
			}
			moyenne_solar = moyenne_solar / NB_VAL;
			moyenne_home = moyenne_home / NB_VAL;
		} else {
			/* Sleep for a litle more than a period (20ms at 50Hz) */
			msleep(23);
			continue;
		}

		/* Feed the dog */
		if ((moyenne_solar != 0) && (moyenne_home != 0)) {
			watchdog_feed();
		}

		/* Get thermocouple value */
		if (1) {
			int ret = 0;
			ret = max31855_sensor_read(&thermo, NULL, &centi_degrees);
			if (ret != 0) {
				uprintf(UART1, "Temp read error : %d\n", ret);
			}
		}
		if (centi_degrees < (FORCE_HEATER_TEMP * 100)) {
			if (forced_heater_mode == 0) {
				uprintf(UART1, "Entering forced mode\n");
				forced_heater_mode = 1;
			}
			status_led(red_on);
			mode = forced;
		} else if ((centi_degrees > (TARGET_FORCED_HEATER_TEMP * 100)) && (forced_heater_mode == 1)) {
			status_led(red_off);
			forced_heater_mode = 0;
			dmx_val = 0;
			uprintf(UART1, "Forced mode exit\n");
			mode = temp_OK;
		}

		/* Do not force if there is some sun, it may be enough to heat again */
		if (moyenne_solar > NO_FORCED_HEATING_ON_SUNNY_DAYS) {
			forced_heater_mode = 0;
			mode = no_heat_prod;
			forced_heater_delay = FORCED_HEATER_DELAY;
		}

		/* Do not force heating if this is an EJP day */
		ejp_in = gpio_read(ejp_in_pin);
		if (ejp_in == DAY_IS_EJP) {
			forced_heater_mode = 0;
			mode = ejp;
		}

		if (never_force == 1) {
			forced_heater_mode = 0;
		}

		/* Did the user request a forced heating ? */
		if (manual_activation_request > 1) {
			forced_heater_mode = 1;
			mode = manual;
			if (manual_activation_request == MANUAL_ACTIVATION_DURATION) {
				uprintf(UART1, "Entering manual forced mode for %d ticks\n", manual_activation_request);
				/* Add a systick callback to handle time counting */
				add_systick_callback(handle_dec_request, DEC_PERIOD);
			}
			if (manual_activation_request < 10) {
				uprintf(UART1, "Leaving manual forced mode\n");
				manual_activation_request = 0;
				remove_systick_callback(handle_dec_request);
			}
		}


		/* Which is the current mode ? */
		if (forced_heater_mode == 1) {
			/* Forced heating mode */
			if ((forced_heater_delay == 0) && (forced_heater_time > 0)) {
				dmx_val = FORCED_MODE_VALUE;
			}
			if (forced_heater_time == 0) {
				forced_heater_delay = FORCED_HEATER_DELAY;
				forced_heater_time = FORCED_HEATER_DURATION;
			}
		} else if (moyenne_solar < moyenne_home) {
			/* Low production mode */
			if (dmx_val > 25) {
				dmx_val -= 25;
			} else {
				dmx_val = 0;
				mode = idle_heat;
			}
			status_led(green_off);
		} else {
			/* High production mode */
			if (dmx_val < 245) {
				dmx_val += 10;
			} else {
				dmx_val = 255;
				mode = full_heat;
			}
			status_led(green_on);
		}

		/* Send DMX frame */
		slots[0] = dmx_val;
		if (slots[0] > 255) {
			slots[0] = 255;
		}
		dmx_send_frame(0x00, slots, 1);
		/* Display */
		if (1) {
			int abs_centi = centi_degrees;
			uprintf(UART1, "%c:%d - Is: %d,%04d - Ih: %d,%04d\n", mode, loop++,
						(moyenne_solar / 1000), (moyenne_solar % 1000),
						(moyenne_home / 1000), (moyenne_home % 1000));
			if (centi_degrees < 0) {
				abs_centi = -centi_degrees;
			}
			uprintf(UART1, "Temp : % 4d.%02d\n", (centi_degrees / 100), (abs_centi % 100));
			uprintf(UART1, "DMX: %d\n\n", slots[0]);
		}
	}
	return 0;
}



