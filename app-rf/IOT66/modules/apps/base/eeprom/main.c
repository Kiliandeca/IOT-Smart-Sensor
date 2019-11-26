/****************************************************************************
 *   apps/base/eeprom/main.c
 *
 *
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
#include "lib/stdio.h"
#include "drivers/i2c.h"
#include "extdrv/eeprom.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/status_led.h"


#ifndef MODULE_SERIAL_NUM
#define MODULE_SERIAL_NUM 42
#endif
#define MODULE_VERSION    0x04
#define MODULE_NAME "GPIO Demo Module"


#define EEPROM_WRITE 1
#undef EEPROM_WRITE

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

const struct pio button_gpio = LPC_GPIO_0_12; /* ISP Used as button */
const struct pio led_gpio = LPC_GPIO_0_4; /* Led toggle on ISP button press */

const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;

const struct pio i2c_eeprom_cs = LPC_GPIO_0_15;


/***************************************************************************** */
/* EEPROM support */

/* Module identification support for DTPlug and DomoTab */

/* The eeprom on the GPIO Demo module must reply on address 0xA8 */
#define EEPROM_ADDR  0xA8

/* Module capabilities */
#define UEXT_MOD_HAS_NONE  0
#define UEXT_MOD_HAS_UART (1 << 0)
#define UEXT_MOD_HAS_I2C  (1 << 1)
#define UEXT_MOD_HAS_SPI  (1 << 2)

struct module_desc {
	uint16_t uart_number;
	uint8_t version;
	uint8_t header_size;
	uint8_t capabilities; /* Bit mask of UEXT_MOD_HAS_* */
	uint8_t name_offset;
	uint8_t name_size;
	uint8_t image_offset;
	uint16_t image_size;
} __attribute__ ((packed));



/* EEPROM Chip select for the GPIO Demo module.
 * Set the SPI SSEL pin low.
 * These functions are specific to the mod_gpio_demo and domotab modules which
 * have access to their onboard EEPROM.
 * It is used to release the gate that blocks the SCL signal to the EEPROM,
 *   allowing multiple eeproms with the same address to be accessed one at a time
 *   on the same I2C Bus, which gives a way to both identify modules presence and
 *   module function, name, and pther capabilities.
 * When the SPI is used as slave, the master has the control of the SPI SSEL signal
 *   and the EEPROM should not be accessed by the module.
 * Other I2C EEPROMs should not need these functions.
 */
int mod_gpio_demo_eeprom_cs_pull_low(void)
{
	/* Configure SPI_CS as output and set it low. */
	config_gpio(&i2c_eeprom_cs, 0, GPIO_DIR_OUT, 0);
	return 0;
}
void mod_gpio_demo_eeprom_cs_release(void)
{
	struct lpc_gpio* gpio_port_regs = LPC_GPIO_REGS(i2c_eeprom_cs.port);
	gpio_port_regs->set = (1 << i2c_eeprom_cs.pin);
}


/* Module description support */

#define DUMP_BUFF_SIZE 80
void module_desc_dump(uint8_t uart_num)
{
	char buff[DUMP_BUFF_SIZE];
	int len = 0, ret = 0;
	struct module_desc desc;

	mod_gpio_demo_eeprom_cs_pull_low();
	/* Read module descriptor structure from eeprom */
	ret = eeprom_read(EEPROM_ADDR, 0, (char*)&desc, sizeof(struct module_desc));
	if (ret != sizeof(struct module_desc)) {
		mod_gpio_demo_eeprom_cs_release();
		uprintf(uart_num, "EEPROM read error\n");
		return;
	}
	/* Get and send the module name */
	if (desc.name_size >= DUMP_BUFF_SIZE) {
		desc.name_size = DUMP_BUFF_SIZE - 1;
	}
	len = eeprom_read(EEPROM_ADDR, desc.name_offset, buff, desc.name_size);
	buff[len] = '\0';

	/* Send the content of the header and the module name */
	uprintf(uart_num, "Module : %s\n", buff);
	uprintf(uart_num, "serial number: %d, version: %d, capabilities: 0x%04x\n",
						desc.uart_number, desc.version, desc.capabilities);
	mod_gpio_demo_eeprom_cs_release();
}

int module_desc_set(char* name, uint8_t module_version, uint16_t serial_num, uint8_t uart_num)
{
	int ret = 0;
	struct module_desc desc = {
		.uart_number = serial_num,
		.version = module_version,
		.capabilities = (UEXT_MOD_HAS_UART | UEXT_MOD_HAS_I2C | UEXT_MOD_HAS_SPI),
		.name_offset = sizeof(struct module_desc),
		.image_offset = 0,
		.image_size = 0,
	};

	mod_gpio_demo_eeprom_cs_pull_low();
	desc.name_size = strlen(name);
	ret = eeprom_write(EEPROM_ADDR, 0, (char*)&desc, sizeof(struct module_desc));
	if (ret != sizeof(struct module_desc)) {
		mod_gpio_demo_eeprom_cs_release();
		uprintf(uart_num, "EEPROM Write error, check eeprom write protection.\n");
		return -1;
	}
	ret += eeprom_write(EEPROM_ADDR, ret, name, desc.name_size);
	mod_gpio_demo_eeprom_cs_release();
	uprintf(uart_num, "EEPROM Write OK.\n");
	return ret;
}


void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_brown_out_detection_config(0); /* No ADC used */
	system_set_default_power_state();
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

	/* Set or read Module identification header in EEPROM */
#ifdef EEPROM_WRITE
	module_desc_set(MODULE_NAME, MODULE_VERSION, MODULE_SERIAL_NUM, 0);
#else
	module_desc_dump(0);
#endif

	while (1) {
		chenillard(250);
	}
	return 0;
}


