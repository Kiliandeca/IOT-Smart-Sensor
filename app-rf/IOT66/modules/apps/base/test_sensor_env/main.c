/****************************************************************************
 *   apps/base/test_sensor_env/main.c
 *
 * E-Xanh Gardener Weather sensors tests using GPIO Demo module
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
#include "drivers/i2c.h"
#include "extdrv/status_led.h"
#include "extdrv/bme280_humidity_sensor.h"
#include "extdrv/veml6070_uv_sensor.h"
#include "extdrv/tsl256x_light_sensor.h"


#define MODULE_VERSION   0x04
#define MODULE_NAME "GPIO Demo"


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
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;


/***************************************************************************** */
/* Luminosity */

/* Note : These are 8bits address */
#define TSL256x_ADDR   0x52 /* Pin Addr Sel (pin2 of tsl256x) connected to GND */
struct tsl256x_sensor_config tsl256x_sensor = {
	.bus_num = I2C0,
	.addr = TSL256x_ADDR,
	.gain = TSL256x_LOW_GAIN,
	.integration_time = TSL256x_INTEGRATION_100ms,
	.package = TSL256x_PACKAGE_T,
};

void lux_config(int uart_num)
{
	int ret = 0;
	ret = tsl256x_configure(&tsl256x_sensor);
	if (ret != 0) {
		uprintf(uart_num, "Lux config error: %d\n", ret);
	}
}

void lux_display(int uart_num)
{
	uint16_t comb = 0, ir = 0;
	uint32_t lux = 0;
	int ret = 0;

	ret = tsl256x_sensor_read(&tsl256x_sensor, &comb, &ir, &lux);
	if (ret != 0) {
		uprintf(uart_num, "Lux read error: %d\n", ret);
	} else {
		uprintf(uart_num, "Lux: %d  (Comb: 0x%04x, IR: 0x%04x)\n", lux, comb, ir);
	}
}



/***************************************************************************** */
/* BME280 Sensor */

/* Note : 8bits address */
#define BME280_ADDR   0xEC
struct bme280_sensor_config bme280_sensor = {
	.bus_num = I2C0,
	.addr = BME280_ADDR,
	.humidity_oversampling = BME280_OS_x16,
	.temp_oversampling = BME280_OS_x16,
	.pressure_oversampling = BME280_OS_x16,
	.mode = BME280_NORMAL,
	.standby_len = BME280_SB_62ms,
	.filter_coeff = BME280_FILT_OFF,
};

void bme_config(int uart_num)
{
	int ret = 0;

	ret = bme280_configure(&bme280_sensor);
	if (ret != 0) {
		uprintf(uart_num, "Sensor config error: %d\n", ret);
	}
}

void bme_display(int uart_num)
{
	uint32_t pressure = 0, temp = 0;
	uint16_t humidity = 0;
	int ret = 0;

	ret = bme280_sensor_read(&bme280_sensor, &pressure, &temp, &humidity);
	if (ret != 0) {
		uprintf(uart_num, "Sensor read error: %d\n", ret);
	} else {
		int comp_temp = 0;
		uint32_t comp_pressure = 0;
		uint32_t comp_humidity = 0;

		comp_temp = bme280_compensate_temperature(&bme280_sensor, temp) / 10;
		comp_pressure = bme280_compensate_pressure(&bme280_sensor, pressure) / 100;
		comp_humidity = bme280_compensate_humidity(&bme280_sensor, humidity) / 10;
		uprintf(uart_num, "P: %d hPa, T: %d,%02d degC, H: %d,%d rH\n",
				comp_pressure,
				comp_temp / 10,  (comp_temp > 0) ? (comp_temp % 10) : ((-comp_temp) % 10),
				comp_humidity / 10, comp_humidity % 10);
	}
}

/***************************************************************************** */
/* UV */

/* The I2C UV light sensor is at addresses 0x70, 0x71 and 0x73 */
/* Note : These are 8bits address */
#define VEML6070_ADDR   0x70
struct veml6070_sensor_config veml6070_sensor = {
	.bus_num = I2C0,
	.addr = VEML6070_ADDR,
};

void uv_config(int uart_num)
{
	int ret = 0;

	/* UV sensor */
	ret = veml6070_configure(&veml6070_sensor);
	if (ret != 0) {
		uprintf(uart_num, "UV config error: %d\n", ret);
	}
}

void uv_display(int uart_num)
{
	uint16_t uv_raw = 0;
	int ret = 0;

	ret = veml6070_sensor_read(&veml6070_sensor, &uv_raw);
	if (ret != 0) {
		uprintf(uart_num, "UV read error: %d\n", ret);
	} else {
		uprintf(uart_num, "UV: 0x%04x\n", uv_raw);
	}
}




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


/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, NULL);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);

	/* Configure environnement sensors */
	bme_config(UART0);
	uv_config(UART0);
	lux_config(UART0);

	while (1) {
		bme_display(UART0);
		uv_display(UART0);
		lux_display(UART0);
		msleep(600);
	}
	return 0;
}



