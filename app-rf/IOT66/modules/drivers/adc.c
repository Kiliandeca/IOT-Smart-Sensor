/****************************************************************************
 *  drivers/adc.c
 *
 * Copyright 2012-2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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
/*                Analog to Digital Converter (ADC)                            */
/***************************************************************************** */

/* ADC driver for the integrated ADC module of the LPC122x.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */

#include "core/system.h"
#include "lib/errno.h"
#include "drivers/adc.h"

/* Should be as near to 9MHz as possible */
#define adc_clk_Val  (9 * 1000 * 1000)



/***************************************************************************** */
/* Generic ADC handler */
void (*adc_int_callback)(uint32_t) = NULL;
void ADC_Handler(void)
{
	volatile struct lpc_adc* adc = LPC_ADC_REGS;
	uint32_t status = adc->status;

	if (adc_int_callback != NULL) {
		adc_int_callback(status);
	}
}

/* Read the conversion from the given channel (0 to 7)
 * This function reads the conversion value directly in the data register and
 * always returns a value.
 * Return 0 if the value is a new one and no overrun occured.
 * Return -EINVAL if channel does not exist
 * Retuen 1 if the value is an old one
 * Return 2 if an overrun occured
 */
int adc_get_value(uint16_t * val, uint8_t channel)
{
	struct lpc_adc* adc = LPC_ADC_REGS;
	uint32_t save_reg = 0;

	if (channel >= NB_ADC_CHANNELS)
		return -EINVAL;

	/* Save the whole register as some bits are cleared when register is read */
	save_reg = adc->data[channel];
	*val = ((save_reg >> LPC_ADC_RESULT_SHIFT) & LPC_ADC_RESULT_MASK);
	/* Has this conversion value been already read ? */
	if (! (save_reg & LPC_ADC_CONV_DONE)) {
		return 1;
	}
	if (save_reg & LPC_ADC_OVERRUN) {
		return 2;
	}
	return 0;
}

/* Start a conversion on the given channel (0 to 7) */
void adc_start_convertion_once(uint8_t channel, uint8_t seq_num, uint8_t use_int)
{
	struct lpc_adc* adc = LPC_ADC_REGS;
	uint32_t reg_val = 0;

	if (channel >= NB_ADC_CHANNELS)
		return;

	/* Get a clean control register */
	reg_val = adc->ctrl & ~(ADC_MCH_MASK | LPC_ADC_START_CONV_MASK | LPC_ADC_BURST);

	/* Set conversion channel bit */
	reg_val |= ADC_MCH(channel);

	/*  Use of interrupts for the specified channel ? */
	if (use_int) {
		/* Set interrupt Bit */
		adc->int_en = ADC_MCH(channel);
	} else {
		adc->int_en = 0;
	}

	/* Start conversion */
	reg_val |= LPC_ADC_START_CONV_NOW;
	adc->ctrl = (reg_val & LPC_ADC_CTRL_MASK);
}


/* Start burst conversions.
 * channels is a bit mask of requested channels.
 * Use ADC_MCH(x) (x = 0 .. 7) for channels selection.
 */
void adc_start_burst_conversion(uint16_t channels, uint8_t seq_num)
{
	struct lpc_adc* adc = LPC_ADC_REGS;
	uint32_t reg_val = 0;

	/* Get a clean control register */
	reg_val = adc->ctrl & ~(ADC_MCH_MASK | LPC_ADC_START_CONV_MASK | LPC_ADC_BURST);

	/* Set conversion channel bits and burst mode */
	reg_val |= channels;
	reg_val |= LPC_ADC_BURST;

	/*  Use of interrupts for the specified channels ? */
	/* FIXME : Need to choose between one single global interrupt or specific interrupts .... */
	/* FIXME : Actually none. */
	adc->int_en = 0;

	/* Start conversion */
	adc->ctrl = (reg_val & LPC_ADC_CTRL_MASK);
}

void adc_stop_burst_conversion(uint8_t seq_num)
{
}


/* This should be used to configure conversion start on falling or rising edges of
 * some signals, or on timer for burst conversions.
 */
void adc_prepare_conversion_on_event(uint16_t channels, uint8_t seq_num, uint8_t event,
										uint8_t use_int, uint32_t mode)
{
	struct lpc_adc* adc = LPC_ADC_REGS;
	uint32_t reg_val = 0;

	/* Get a clean control register */
	reg_val = adc->ctrl & ~(ADC_MCH_MASK | LPC_ADC_START_CONV_MASK | LPC_ADC_BURST);
	/* Set conversion channel bits and burst mode */
	reg_val |= (channels & ADC_MCH_MASK);
	/* Set conversion condition bits */
	reg_val |= LPC_ADC_START_CONV_EVENT(event);
	if (mode != 0) {
		reg_val |= (mode & LPC_ADC_ADDITIONAL_MODE_MASK);
	}

	/*  Use of interrupts for the specified channel ? */
	if (use_int) {
		/* FIXME : Need to choose between one single global interrupt or specific interrupts .... */
	} else {
		adc->int_en = 0;
	}

	/* Enable conversion on selected event */
	adc->ctrl = (reg_val & LPC_ADC_CTRL_MASK);
}


/* Software trigger of the given configured sequence */
void adc_trigger_sequence_conversion(uint8_t seq_num)
{
	struct lpc_adc* adc = LPC_ADC_REGS;

	adc->ctrl &= ~(LPC_ADC_START_CONV_MASK);
	adc->ctrl |= LPC_ADC_START_CONV_NOW;
}


/***************************************************************************** */
/*   ADC Setup : private part : Clocks, Power and Mode   */

void adc_clk_update(void)
{
	struct lpc_adc* adc = LPC_ADC_REGS;
	uint32_t main_clock = get_main_clock();
	uint32_t clkdiv = 0;

	/* Configure ADC clock to get the 9MHz sample clock */
	clkdiv = (main_clock / adc_clk_Val);
	adc->ctrl |= ((clkdiv & 0xFF) << 8);
}


void adc_on(void (*adc_callback)(uint32_t))
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct lpc_adc* adc = LPC_ADC_REGS;

	/* Disable ADC Interrupt */
	NVIC_DisableIRQ(ADC_IRQ);

	/* Brown-Out detection must be powered to operate the ADC.
	 * See Section 19.2 of UM10441 revision 2.1 or newer for more information */
	sys_config->powerdown_run_cfg &= ~LPC_POWER_DOWN_BOD;

	/* Power-up ADC */
	sys_config->powerdown_run_cfg &= ~LPC_POWER_DOWN_ADC;
	/* Provide clock to ADC */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_ADC, 1);
	adc_clk_update();

	/* Prevent unconfigured conversion start */
	adc->ctrl &= ~LPC_ADC_START_CONV_MASK;

	/* Remove the default global interrupt enabled setting */
	adc->int_en = 0;
	/* Register a possible calback */
	adc_int_callback = adc_callback;

	/* Enable ADC Interrupt */
	NVIC_EnableIRQ(ADC_IRQ);
}

void adc_off(void)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;

	/* Disable ADC Interrupt */
	NVIC_DisableIRQ(ADC_IRQ);
	/* Remove callback */
	adc_int_callback = NULL;
	/* Power Down ADC */
	sys_config->powerdown_run_cfg |= LPC_POWER_DOWN_ADC;
	/* Remove clock from ADC block */
	subsystem_power(LPC_SYS_ABH_CLK_CTRL_ADC, 0);
}

