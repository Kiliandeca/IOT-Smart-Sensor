/****************************************************************************
 *   apps/base/servomotor/main.c
 *
 * Servo-motor example
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
#include "extdrv/status_led.h"
#include "drivers/adc.h"
#include "drivers/timers.h"


#define MODULE_VERSION   0x04
#define MODULE_NAME "GPIO Demo Module"


#define SELECTED_FREQ  FREQ_SEL_48MHz

/* Chose on of these depending on the signal you need:
 *  - inverted one (3), for use with a single transistor
 *  - non inverted (40 - 3), for use with two transistors (or none).
 */
#define DUTY_INVERTED 1
#if (DUTY_INVERTED == 1)
#define SERVO_MED_POS_DUTY_CYCLE  3
#else
#define SERVO_MED_POS_DUTY_CYCLE  (40 - 3)
#endif

#define LPC_TIMER_PIN_CONFIG   (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL | LPC_IO_DRIVE_HIGHCURENT)

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
	/* TIMER_32B0 */
	{ LPC_TIMER_32B0_M1_PIO_0_19, LPC_TIMER_PIN_CONFIG },
	ARRAY_LAST_PIO,
};

const struct pio_config adc_pins[] = {
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	{ LPC_ADC_AD3_PIO_1_1,  LPC_IO_ANALOG },
	{ LPC_ADC_AD4_PIO_1_2,  LPC_IO_ANALOG },
	{ LPC_ADC_AD5_PIO_1_3,  LPC_IO_ANALOG },
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

/***************************************************************************** */
static uint32_t servo_med_pos_cmd = 0;
static uint32_t servo_one_deg_step = 0;
static uint8_t timer = 0;
static uint8_t channel = 0;
int servo_config(uint8_t timer_num, uint8_t pwm_chan, uint8_t uart_num)
{
	uint32_t servo_command_period = 0;
	struct lpc_timer_pwm_config timer_conf = {
		.nb_channels = 1,
		.period_chan = 3,
	};

	if (timer_num > LPC_TIMER_32B1) {
		uprintf(uart_num, "Bad timer number\n");
		return -1;
	}
	if (pwm_chan >= 3) {
		uprintf(uart_num, "Bad channel number\n");
		return -1;
	}
	timer = timer_num;
	channel = pwm_chan;
	timer_conf.outputs[0] = pwm_chan;

	/* compute the period and median position for the servo command */
	/* We want 20ms (50Hz), timer counts at main clock frequency */
	servo_command_period = get_main_clock() / 50;
	/* servo_command_period is 20ms, we need 1.5ms, which is 3/40. */
	servo_med_pos_cmd = ((servo_command_period / 40) * SERVO_MED_POS_DUTY_CYCLE);
	servo_one_deg_step = ((servo_command_period / 41) / 48);
	timer_conf.match_values[0] = servo_med_pos_cmd;
	timer_conf.period = servo_command_period;

	timer_on(timer, 0, NULL);
	timer_pwm_config(timer, &timer_conf);
	timer_start(timer);

	uprintf(uart_num, "Servos configured (T%d : C%d), Period : %d, med_pos : %d\n",
						timer, channel, servo_command_period, servo_med_pos_cmd);

	return 0;
}

int set_servo(int adc_num, int uart_num)
{
	uint16_t val = 0, angle = 0;
	uint32_t pos = servo_med_pos_cmd;

	adc_start_convertion_once(adc_num, LPC_ADC_SEQ(0), 0);
	msleep(10);
	adc_get_value(&val, adc_num);
	uprintf(uart_num, "ADC(%d): %d (raw: 0x%04x)\n", adc_num, val, val);

	angle = ((val * 10) / 56);
	if (angle > 180) {
		angle = 180;
	}

	/* And compute the new match value for the angle */
	if (angle >= 90) {
		pos += ((angle - 90) * servo_one_deg_step);
	} else {
		pos -= ((90 - angle) * servo_one_deg_step);
	}
	timer_set_match(timer, channel, pos);
	uprintf(uart_num, "Servo(%d): %d (%d)\n", channel, angle, pos);
	return val;
}


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



/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, NULL);
	adc_on(NULL);
	servo_config(LPC_TIMER_32B0, 1, 0);

	while (1) {
		chenillard(500);
		/* ADC Test */
		set_servo(LPC_ADC(0), 0);
	}
	return 0;
}



