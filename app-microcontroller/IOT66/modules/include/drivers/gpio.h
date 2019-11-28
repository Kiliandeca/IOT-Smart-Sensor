/****************************************************************************
 *  drivers/gpio.h
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

#ifndef DRIVERS_GPIO_H
#define DRIVERS_GPIO_H


#include "lib/stdint.h"
#include "core/lpc_regs.h"
#include "core/pio.h"


/***************************************************************************** */
/*                     General Purpose Input/Output                            */
/***************************************************************************** */
/* General Purpose Input/Output (GPIO) */
struct lpc_gpio
{
	volatile uint32_t mask;       /* 0x00 : Pin mask, affects in, out, set, clear and invert */
	volatile uint32_t in;         /* 0x04 : Port data Register (R/-) */
	volatile uint32_t out;        /* 0x08 : Port output Register (R/W) */
	volatile uint32_t set;        /* 0x0C : Port output set Register (-/W) */
	volatile uint32_t clear;      /* 0x10 : Port output clear Register (-/W) */
	volatile uint32_t toggle;     /* 0x14 : Port output invert Register (-/W) */
	uint32_t reserved_1[2];
	volatile uint32_t data_dir;   /* 0x20 : Data direction Register (R/W) */
	volatile uint32_t int_sense;  /* 0x24 : Interrupt sense Register (R/W) */
	volatile uint32_t int_both_edges; /* 0x28 : Interrupt both edges Register (R/W) */
	volatile uint32_t int_event;  /* 0x2C : Interrupt event Register  (R/W) */
	volatile uint32_t int_enable; /* 0x30 : Interrupt mask Register (R/W) */
	volatile uint32_t raw_int_status;    /* 0x34 : Raw interrupt status Register (R/-) */
	volatile uint32_t masked_int_status; /* 0x38 : Masked interrupt status Register (R/-) */
	volatile uint32_t int_clear;  /* 0x3C : Interrupt clear Register (R/W) */
};
#define LPC_GPIO_0      ((struct lpc_gpio *) LPC_GPIO_0_BASE)
#define LPC_GPIO_1      ((struct lpc_gpio *) LPC_GPIO_1_BASE)
#define LPC_GPIO_2      ((struct lpc_gpio *) LPC_GPIO_2_BASE)

#define LPC_GPIO_REGS(x)  ((struct lpc_gpio *) (LPC_AHB_BASE + (0x10000 * (x))))



/***************************************************************************** */
/*   Public access to GPIO setup   */

enum gpio_ports {
	GPIO_PORT0 = 0,
	GPIO_PORT1,
	GPIO_PORT2,
};

#define GPIO_DIR_IN 0
#define GPIO_DIR_OUT 1

enum gpio_interrupt_senses {
	EDGES_BOTH = 0,
	EDGE_FALLING,
	EDGE_RISING,
	LEVEL_HIGH,
	LEVEL_LOW,
};



/* GPIO Activation */
void gpio_on(void);

void gpio_off(void);

#define gpio_dir_in(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->data_dir &= ~(1 << gpio.pin);\
}

#define gpio_dir_out(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->data_dir |= (1 << gpio.pin);\
}


#define gpio_set(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->set = (1 << gpio.pin);\
}

#define gpio_clear(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->clear = (1 << gpio.pin);\
}

#define gpio_toggle(gpio) \
{ \
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port); \
	gpio_port->toggle = (1 << gpio.pin);\
}

static inline uint32_t gpio_read(const struct pio gpio)
{
	struct lpc_gpio* gpio_port = LPC_GPIO_REGS(gpio.port);
	return (gpio_port->in & (1 << gpio.pin));
}


/* GPIO Configuration
 * This function calls the config_pio() function for the gpio with the given
 * mode, configures the direction of the pin and sets the initial state.
 * Use GPIO_DIR_IN or GPIO_DIR_OUT for the direction "dir", and 0 or 1 for the initial
 * value "ini_val".
 */
void config_gpio(const struct pio* gpio, uint32_t mode, uint8_t dir, uint8_t ini_val);


/* GPIO Interrupts */
/* Add a callback on a GPIO interrupt.
 * This call will configure the GPIO (call to config_pio()), set it as input and
 * activate the interrupt on the given 'sense' event. use the gpio_interrupt_senses
 * enum for 'sense' values.
 * The callback will receive the pin number as argument (but not the port number).
 * Note :
 *   The interrupt hanlers are not efficient if the pin is not a low numbered one (require
 *   32 shifts + test for pin number 31).
 *   Use them if you place the signals generating interrupts on low numbered pins.
 *   When possible, get in touch with the people doing the electronic design, or change
 *   the handlers in drivers/gpio.c
 */
int set_gpio_callback(void (*callback) (uint32_t), const struct pio* gpio, uint8_t sense);

void remove_gpio_callback(const struct pio* gpio);




#endif /* DRIVERS_GPIO_H */
