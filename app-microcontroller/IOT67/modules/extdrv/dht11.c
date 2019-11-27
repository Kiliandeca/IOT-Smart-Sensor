/****************************************************************************
 *   extdrv/dht11.c
 *
 *
 *
 * Copyright 2013 Nathael Pajani <nathael.pajani@ed3l.fr>
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

/*
 * Support for the DHT11 Temperature and Humidity sensor
 * Remember that the sensor operates on 3.5 to 5.5V, and that a level shifter must
 *   be used to interface with the DHT11 sensor.
 *
 * Note : This support uses a lot of "sleep()" calls, and is thus very ineficient
 *   and many data conversion can occur (checksum errors, though on all of my tests
 *   the errors were always on the lower bit, when the temp and humidity values
 *   were changing, and the scope always reported very stable signals, so I suspect
 *   that the error is not on the micro-controler side ... )
 * The addition of the "epsilon_error" with a value of 1 removed all errors on my
 *   side., while still getting coherent values.
 *
 * DHT11 protocol can be found here : http://embedded-lab.com/blog/?p=4333
 *
 */

#include "core/system.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/gpio.h"
#include "drivers/serial.h"
#include "extdrv/status_led.h"


static struct pio dht11_gpio;
void dht11_config(const struct pio* gpio)
{
	/* Configure as output and set it high. */
	/* This is the "do nothing" state */
	config_gpio(gpio, LPC_IO_MODE_PULL_UP, GPIO_DIR_OUT, 1);
	pio_copy(&dht11_gpio, gpio);
}

static unsigned char dht11_read_dat()
{
	struct lpc_gpio* gpio_port_regs = LPC_GPIO_REGS(dht11_gpio.port);
	int i = 0;
	unsigned char val = 0;
	for (i = 0; i < 8; i++) {
		/* Wait end of 'low' */
		while(!(gpio_port_regs->in & (1 << dht11_gpio.pin)));

		/* Wait 35us */
		usleep(35);

		/* read one bit */
		if (gpio_port_regs->in & (1 << dht11_gpio.pin)) {
			val |= (1 << (7-i));
		}

		/* Wait end of bit */
		while (gpio_port_regs->in & (1 << dht11_gpio.pin));
	}
	return val;
}

void dht11_display(int serial_num, int epsilon_error)
{
	struct lpc_gpio* gpio_port_regs = LPC_GPIO_REGS(dht11_gpio.port);
	unsigned char data[5];
	unsigned char checksum = 0;
	int err = 0, i = 0;

	/* Set pin as output */
	gpio_port_regs->data_dir |= (1 << dht11_gpio.pin);

	/* Send the "start" bit : low pulse of more than 18ms */
	gpio_port_regs->clear = (1 << dht11_gpio.pin);
	msleep(25);
	gpio_port_regs->set = (1 << dht11_gpio.pin);

	/* Set pin as input */
	gpio_port_regs->data_dir &= ~(1 << dht11_gpio.pin);

	/* Wait for start condition ack : 80us low followed by 80us high.
	 * Do not care about the durations */
	while (gpio_port_regs->in & (1 << dht11_gpio.pin));
	while (!(gpio_port_regs->in & (1 << dht11_gpio.pin)));
	while (gpio_port_regs->in & (1 << dht11_gpio.pin));

	/* Start reading data : 40 bits */
	for (i = 0; i < 5; i++){
	    data[i] = dht11_read_dat();
		if (i < 4) {
			checksum += data[i];
		}
    }

	if (checksum != data[4]) {
		if (epsilon_error == 0) {
			err = 1;
		} else if ((checksum < (data[4] - epsilon_error)) && (checksum > (data[4] + epsilon_error))) {
			err = 1;
		}
	}
	if (err == 1) {
		status_led(red_only);
		uprintf(serial_num, "TH_ERR - H: 0x%02x,0x%02x - T: 0x%02x,0x%02x - C: 0x%02x\r\n",
								 data[0], data[1], data[2], data[3], data[4]);
		return;
	}

	uprintf(serial_num, "H: %d,%d - T: %d,%d\r\n", data[0], data[1], data[2], data[3]);
	status_led(green_only);
}
