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
#include "extdrv/status_led.h"


#define MODULE_VERSION    0x01
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
	{ LPC_ADC_AD4_PIO_1_2,  LPC_IO_ANALOG },
	/* GPIO */
	{ LPC_GPIO_0_0, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Pump */
	{ LPC_GPIO_0_4, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Humidity Sensors Power control */
	{ LPC_GPIO_0_5, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Motor Power control */
	{ LPC_GPIO_0_6, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Led strip */
	{ LPC_GPIO_0_7, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) },  /* Bluetooth Power control */
	{ LPC_GPIO_0_12, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* ISP Button */
	{ LPC_GPIO_0_18, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Level Sensor Power control */
	{ LPC_GPIO_0_22, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 0 */
	{ LPC_GPIO_0_23, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 1 */
	{ LPC_GPIO_0_24, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 2 */
	{ LPC_GPIO_0_25, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 3 */
	{ LPC_GPIO_0_26, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 4 */
	{ LPC_GPIO_0_27, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Position Sensor 5 */
	{ LPC_GPIO_0_28, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Motor Enable */
	{ LPC_GPIO_0_29, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Motor Direction control */
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio button = LPC_GPIO_0_12; /* ISP button */

const struct pio ws2812_data_out_pin = LPC_GPIO_0_19; /* Led control data pin */

const struct pio motor_power = LPC_GPIO_0_5; /* Motor Power */
const struct pio motor_enable = LPC_GPIO_0_28; /* Motor Enable */
const struct pio motor_direction = LPC_GPIO_0_29; /* Motor Direction */

const struct pio pompe_on = LPC_GPIO_0_0; /* Water Pump */


#define NB_POS_SENSORS   6
const struct pio positions[NB_POS_SENSORS] = {
	LPC_GPIO_0_22,
	LPC_GPIO_0_23,
	LPC_GPIO_0_24,
	LPC_GPIO_0_25,
	LPC_GPIO_0_26,
	LPC_GPIO_0_27,
};

/***************************************************************************** */
void system_init()
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


volatile uint8_t current_position = 255;
void pos_detect(uint32_t gpio)
{
	if (gpio_read(positions[(gpio - 22)]) == 0) {
		/* The sensor output is 0 when it detects a magnet */
		current_position = gpio - 22;
	} else {
		current_position = gpio - 12;
	}
}

volatile uint8_t motor_sens = 2;
volatile uint8_t pompe_cmd = 0;
volatile uint8_t motor_move_req = 0;
void handle_cmd(uint8_t c)
{
	switch (c) {
		case '1':
			motor_sens = 1;
			uprintf(UART0, "Motor forward\n");
			break;
		case '2':
			motor_sens = 2;
			uprintf(UART0, "Motor backward\n");
			break;
		case 'p':
			pompe_cmd = 1;
			break;
		case 'm':
			motor_move_req = 1;
			break;
		default :
			uprintf(UART0, "All Stop request\n");
			motor_sens = 0;
			pompe_cmd = 0;
	}

}
volatile uint8_t motor_status = 0;
volatile uint32_t debug = 0;
void motor_on(uint32_t gpio)
{
	if (gpio != button.pin) {
		motor_status = 5;
		debug = gpio;
		return;
	}
	if ((gpio_read(button) == 0) && (motor_sens != 0)) {
		/* Turn motor ON */
		gpio_set(motor_power);
		gpio_set(motor_enable);
		if (motor_sens == 1) {
			motor_status = 1;
			gpio_set(motor_direction);
		} else {
			motor_status = 2;
			gpio_clear(motor_direction);
		}
	} else {
		motor_status = 0;
		gpio_clear(motor_power);
		gpio_clear(motor_enable);
	}
}

/***************************************************************************** */
int main(void)
{
	int i = 0;
	uint8_t old_position = 255;
	uint8_t old_status = 0, old_pompe = 0;
	system_init();
	uart_on(UART0, 115200, handle_cmd);

	set_gpio_callback(motor_on, &button, EDGES_BOTH);

	/* Selector position sensors */
	for (i = 0; i < NB_POS_SENSORS; i++) {
		set_gpio_callback(pos_detect, &(positions[i]), EDGES_BOTH);
	}

	/* GPIO for Pump control */
	config_gpio(&pompe_on, 0, GPIO_DIR_OUT, 0);

	/* GPIO for Motor control */
	config_gpio(&motor_power, 0, GPIO_DIR_OUT, 0);
	config_gpio(&motor_enable, 0, GPIO_DIR_OUT, 0);
	config_gpio(&motor_direction, 0, GPIO_DIR_OUT, 0);

	gpio_clear(motor_power);
	gpio_clear(motor_enable);
	uprintf(UART0, "E-Xanh Gardener started\n");

	while (1) {
		chenillard(25);
		if (current_position != old_position) {
			old_position = current_position;
			uprintf(UART0, "New position : %d\n", current_position);
			old_position = current_position;
		}
		if (old_pompe != pompe_cmd) {
			if (pompe_cmd == 1) {
				uprintf(UART0, "Pump On\n");
				gpio_set(pompe_on);
			} else {
				uprintf(UART0, "Pump Stopped\n");
				gpio_clear(pompe_on);
			}
			old_pompe = pompe_cmd;
		}
		if (motor_move_req == 1) {
			motor_move_req = 0;
			/* Turn motor ON for a short time */
			gpio_set(motor_power);
			gpio_set(motor_enable);
			if (motor_sens == 1) {
				uprintf(UART0, "Motor moving forward for 250ms\n");
				gpio_set(motor_direction);
			} else {
				uprintf(UART0, "Motor moving backward for 250ms\n");
				gpio_clear(motor_direction);
			}
			msleep(250);
			/* And turn motor back Off */
			gpio_clear(motor_power);
			gpio_clear(motor_enable);
		}
		if (old_status != motor_status) {
			switch (motor_status) {
				case 0:
					uprintf(UART0, "Motor off\n");
					break;
				case 1:
					uprintf(UART0, "Motor moving forward\n");
					break;
				case 2:
					uprintf(UART0, "Motor moving backward\n");
					break;
				case 5:
					uprintf(UART0, "Motor error: 0x%08x\n", debug);
					break;
			}
			old_status = motor_status;
		}
	}
	return 0;
}



