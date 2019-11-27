/****************************************************************************
 *  core/pio.c
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
/*                GPIOs                                                        */
/***************************************************************************** */

/*   Public access to Pins setup
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */


#include "core/system.h"
#include "core/pio.h"



/***************************************************************************** */
static volatile uint32_t* pio_regs_handles_port0[PORT0_NB_PINS] = {
	&(LPC_IO_CONTROL->pio0_0),
	&(LPC_IO_CONTROL->pio0_1),
	&(LPC_IO_CONTROL->pio0_2),
	&(LPC_IO_CONTROL->pio0_3),
	&(LPC_IO_CONTROL->pio0_4),
	&(LPC_IO_CONTROL->pio0_5),
	&(LPC_IO_CONTROL->pio0_6),
	&(LPC_IO_CONTROL->pio0_7),
	&(LPC_IO_CONTROL->pio0_8),
	&(LPC_IO_CONTROL->pio0_9),
	&(LPC_IO_CONTROL->pio0_10),
	&(LPC_IO_CONTROL->pio0_11),
	&(LPC_IO_CONTROL->pio0_12),
	&(LPC_IO_CONTROL->pio0_13),
	&(LPC_IO_CONTROL->pio0_14),
	&(LPC_IO_CONTROL->pio0_15),
	&(LPC_IO_CONTROL->pio0_16),
	&(LPC_IO_CONTROL->pio0_17),
	&(LPC_IO_CONTROL->pio0_18),
	&(LPC_IO_CONTROL->pio0_19),
	&(LPC_IO_CONTROL->pio0_20),
	&(LPC_IO_CONTROL->pio0_21),
	&(LPC_IO_CONTROL->pio0_22),
	&(LPC_IO_CONTROL->pio0_23),
	&(LPC_IO_CONTROL->pio0_24),
	&(LPC_IO_CONTROL->pio0_25),
	&(LPC_IO_CONTROL->pio0_26),
	&(LPC_IO_CONTROL->pio0_27),
	&(LPC_IO_CONTROL->pio0_28),
	&(LPC_IO_CONTROL->pio0_29),
	&(LPC_IO_CONTROL->pio0_30),
	&(LPC_IO_CONTROL->pio0_31),
};
static volatile uint32_t* pio_regs_handles_port1[PORT1_NB_PINS] = {
	&(LPC_IO_CONTROL->pio1_0),
	&(LPC_IO_CONTROL->pio1_1),
	&(LPC_IO_CONTROL->pio1_2),
	&(LPC_IO_CONTROL->pio1_3),
	&(LPC_IO_CONTROL->pio1_4),
	&(LPC_IO_CONTROL->pio1_5),
	&(LPC_IO_CONTROL->pio1_6),
};
static volatile uint32_t* pio_regs_handles_port2[PORT2_NB_PINS] = {
	&(LPC_IO_CONTROL->pio2_0),
	&(LPC_IO_CONTROL->pio2_1),
	&(LPC_IO_CONTROL->pio2_2),
	&(LPC_IO_CONTROL->pio2_3),
	&(LPC_IO_CONTROL->pio2_4),
	&(LPC_IO_CONTROL->pio2_5),
	&(LPC_IO_CONTROL->pio2_6),
	&(LPC_IO_CONTROL->pio2_7),
	&(LPC_IO_CONTROL->pio2_8),
	&(LPC_IO_CONTROL->pio2_9),
	&(LPC_IO_CONTROL->pio2_10),
	&(LPC_IO_CONTROL->pio2_11),
	&(LPC_IO_CONTROL->pio2_12),
	&(LPC_IO_CONTROL->pio2_13),
	&(LPC_IO_CONTROL->pio2_14),
	&(LPC_IO_CONTROL->pio2_15),
};

/* Simple copy function. */
void pio_copy(struct pio* dst, const struct pio* src)
{
	if ((dst == NULL) || (src == NULL)) {
		return;
	}
	dst->port = src->port;
	dst->pin = src->pin;
	dst->alt_setting = src->alt_setting;
}

/* Configure the pin in the requested function and mode. */
void config_pio(const struct pio* pp, uint32_t mode)
{
	volatile uint32_t* handle = NULL;

	if (pp == NULL) {
		return;
	}
	switch (pp->port) {
		case 0:
			if (pp->pin >= PORT0_NB_PINS)
				return;
			handle = pio_regs_handles_port0[pp->pin];
			break;
		case 1:
			if (pp->pin >= PORT1_NB_PINS)
				return;
			handle = pio_regs_handles_port1[pp->pin];
			break;
		case 2:
			if (pp->pin >= PORT2_NB_PINS)
				return;
			handle = pio_regs_handles_port2[pp->pin];
			break;
		default:
			return;
	}
	/* Make sure IO_Config is clocked */
	io_config_clk_on();

	*handle = (LPC_IO_FUNC_ALT(pp->alt_setting) | mode);

	/* Config done, power off IO_CONFIG block */
	io_config_clk_off();
}


void set_pins(const struct pio_config* pins)
{
	int i = 0;
	for (i = 0; pins[i].pio.port != 0xFF; i++) {
		config_pio(&(pins[i].pio), pins[i].mode);
	}
}

/* IO config clock */
/* To change GPIO config the io config block must be powered on */
void io_config_clk_on(void)
{
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_IO_CONFIG, 1);
}
void io_config_clk_off(void)
{
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_IO_CONFIG, 0);
}

