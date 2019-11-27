/****************************************************************************
 *  drivers/gpio.c
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
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

/***************************************************************************** */
/*                GPIOs and GPIO Interrupts                                    */
/***************************************************************************** */

/* Driver for GPIO configuration and access (including GPIO interrupts) on the LPC122x.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */


#include "core/system.h"
#include "lib/errno.h"
#include "core/pio.h"
#include "drivers/gpio.h"



/***************************************************************************** */
/*   GPIO setup   */

void gpio_on(void)
{
	/* Provide power to GPIO control blocks */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO0, 1);
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO1, 1);
	/* FIXME : Power this one too if you use LQFP64 or LQFP100 packages */
	/* subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO2, 1); */
}
void gpio_off(void)
{
	/* Remove power from GPIO control blocks */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO0, 0);
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO1, 0);
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_GPIO2, 0);
}

/*
 * This function calls the config_pio() function for the gpio with the given
 * mode, configures the direction of the pin and sets the initial state.
 */
void config_gpio(const struct pio* gpio, uint32_t mode, uint8_t dir, uint8_t ini_val)
{
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio->port);

	/* Configure as GPIO */
	config_pio(gpio, mode | LPC_IO_DIGITAL);

	if (dir == GPIO_DIR_IN) {
		gpio_port->data_dir &= ~(1 << gpio->pin);
	} else {
		gpio_port->data_dir |= (1 << gpio->pin);
		if (ini_val == 0) {
			gpio_port->clear = (1 << gpio->pin);
		} else {
			gpio_port->set = (1 << gpio->pin);
		}
	}
}

/***************************************************************************** */
/* GPIO Interrupts Callbacks */
static void (*gpio_callbacks_port0[PORT0_NB_PINS]) (uint32_t);
static void (*gpio_callbacks_port1[PORT1_NB_PINS]) (uint32_t);
static void (*gpio_callbacks_port2[PORT2_NB_PINS]) (uint32_t);

int set_gpio_callback(void (*callback) (uint32_t), const struct pio* gpio, uint8_t sense)
{
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio->port);
	uint32_t irq = 0;

	/* Register the callback */
	/* FIXME : we should check that there were none registered for the selected pin */
	switch (gpio->port) {
		case 0:
			if (gpio->pin >= PORT0_NB_PINS)
				return -EINVAL;
			gpio_callbacks_port0[gpio->pin] = callback;
			irq = PIO_0_IRQ;
			break;
		case 1:
			if (gpio->pin >= PORT1_NB_PINS)
				return -EINVAL;
			gpio_callbacks_port1[gpio->pin] = callback;
			irq = PIO_1_IRQ;
			break;
		case 2:
			if (gpio->pin >= PORT2_NB_PINS)
				return -EINVAL;
			gpio_callbacks_port2[gpio->pin] = callback;
			irq = PIO_2_IRQ;
			break;
		default:
			return -EINVAL;
	}

	/* Configure the pin as interrupt source */
	gpio_port->data_dir &= ~(1 << gpio->pin); /* Input */
	config_pio(gpio, LPC_IO_DIGITAL);
	switch (sense) {
		case EDGES_BOTH:
			gpio_port->int_sense &= ~(1 << gpio->pin);
			gpio_port->int_both_edges |= (1 << gpio->pin);
			break;
		case EDGE_RISING:
			gpio_port->int_sense &= ~(1 << gpio->pin);
			gpio_port->int_both_edges &= ~(1 << gpio->pin);
			gpio_port->int_event |= (1 << gpio->pin);
			break;
		case EDGE_FALLING:
			gpio_port->int_sense &= ~(1 << gpio->pin);
			gpio_port->int_both_edges &= ~(1 << gpio->pin);
			gpio_port->int_event &= ~(1 << gpio->pin);
			break;
		case LEVEL_LOW:
			gpio_port->int_sense |= (1 << gpio->pin);
			gpio_port->int_both_edges &= ~(1 << gpio->pin);
			gpio_port->int_event &= ~(1 << gpio->pin);
			break;
		case LEVEL_HIGH:
			gpio_port->int_sense |= (1 << gpio->pin);
			gpio_port->int_both_edges &= ~(1 << gpio->pin);
			gpio_port->int_event |= (1 << gpio->pin);
			break;
		default: /* Not handled, do not activate the interrupt */
			return -EINVAL;
	}
	/* Clear edge detection logic */
	gpio_port->int_clear |= (1 << gpio->pin);
	/* Enable interrupt */
	gpio_port->int_enable |= (1 << gpio->pin);
	NVIC_EnableIRQ(irq);
	return 0;
}

void remove_gpio_callback(const struct pio* gpio)
{
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio->port);
	uint32_t irq = 0;

	/* Remove the handler */
	switch (gpio->port) {
		case 0:
			if (gpio->pin >= PORT0_NB_PINS)
				return;
			gpio_callbacks_port0[gpio->pin] = NULL;
			irq = PIO_0_IRQ;
			break;
		case 1:
			if (gpio->pin >= PORT1_NB_PINS)
				return;
			gpio_callbacks_port1[gpio->pin] = NULL;
			irq = PIO_1_IRQ;
			break;
		case 2:
			if (gpio->pin >= PORT2_NB_PINS)
				return;
			gpio_callbacks_port2[gpio->pin] = NULL;
			irq = PIO_2_IRQ;
			break;
		default:
			return;
	}
	/* And disable the interrupt */
	gpio_port->int_enable &= ~(1 << gpio->pin);
	if (gpio_port->int_enable == 0) {
		NVIC_DisableIRQ(irq);
	}
}


/* Interrupt Handlers */
/* Those handlers are far from the most effective if used without concertation
 * with the people doing the electronic design.
 * Use them if you place the signals generating interrupts on low numbered pins
 */
void PIO_0_Handler(void)
{
	struct lpc_gpio* gpio0 = LPC_GPIO_0;
	uint32_t status = gpio0->masked_int_status;
	uint32_t i = 0;

	/* Call interrupt handlers */
	while (status) {
		if (status & 1) {
			/* Is there an handler for this one ? */
			if (gpio_callbacks_port0[i] != NULL) {
				gpio_callbacks_port0[i](i);
			}
			/* Clear edge detection logic */
			gpio0->int_clear |= (1 << i);
		}
		status >>= 1;
		i++;
	}
}
void PIO_1_Handler(void)
{
	struct lpc_gpio* gpio1 = LPC_GPIO_1;
	uint32_t status = gpio1->masked_int_status;
	uint32_t i = 0;

	/* Call interrupt handlers */
	while (status) {
		if (status & 1) {
			/* Is there an handler for this one ? */
			if (gpio_callbacks_port1[i] != NULL) {
				gpio_callbacks_port1[i](i);
			}
			/* Clear pending interrupt */
			gpio1->int_clear |= (1 << i);
		}
		status >>= 1;
		i++;
	}
}
void PIO_2_Handler(void)
{
	struct lpc_gpio* gpio2 = LPC_GPIO_2;
	uint32_t status = gpio2->masked_int_status;
	uint32_t i = 0;

	/* Call interrupt handlers */
	while (status) {
		if (status & 1) {
			/* Is there an handler for this one ? */
			if (gpio_callbacks_port2[i] != NULL) {
				gpio_callbacks_port2[i](i);
			}
			/* Clear pending interrupt */
			gpio2->int_clear |= (1 << i);
		}
		status >>= 1;
		i++;
	}
}


