/****************************************************************************
 *  drivers/adc.h
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

#ifndef DRIVERS_ADC_H
#define DRIVERS_ADC_H


#include "lib/stdint.h"
#include "core/lpc_regs.h"

/***************************************************************************** */
/*                Analog to Digital Converter (ADC)                            */
/***************************************************************************** */

/* ADC driver for the integrated ADC module of the LPC122x.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */

#define NB_ADC_CHANNELS  8
#define NB_ADC_SEQUENCES 1

#define LPC_ADC_SEQ(x)  0


/* Read the conversion from the given channel
 * This function reads the conversion value directly in the data register and
 * always returns a value.
 * Return 0 if the value is a new one and no overrun occured.
 * Return -EINVAL if channel does not exist
 * Retuen 1 if the value is an old one
 * Return 2 if an overrun occured
 */
int adc_get_value(uint16_t * val, uint8_t channel);

/* Start a conversion on the given channel (0 to 7)
 * Unsupported yet : Set use_int to 1 to have your interrupt callback called upon conversion done.
 */
void adc_start_convertion_once(uint8_t channel, uint8_t seq_num, uint8_t use_int);

/* Start burst conversions.
 * channels is a bit mask of requested channels.
 * Use ADC_MCH(x) (x = 0 .. 7) for channels selection.
 */
void adc_start_burst_conversion(uint16_t channels, uint8_t seq_num);
void adc_stop_burst_conversion(uint8_t seq_num);


enum lpc_adc_start_conv_events {
	LPC_ADC_START_CONV_EDGE_CT16B0_CAP0 = 2,
	LPC_ADC_START_CONV_EDGE_CT32B0_CAP0,
	LPC_ADC_START_CONV_EDGE_CT32B0_MAT0,
	LPC_ADC_START_CONV_EDGE_CT32B0_MAT1,
	LPC_ADC_START_CONV_EDGE_CT16B0_MAT0,
	LPC_ADC_START_CONV_EDGE_CT16B0_MAT1,
};
/* This should be used to configure conversion start on falling or rising edges of
 * some signals, or on timer for burst conversions.
 */
void adc_prepare_conversion_on_event(uint16_t channels, uint8_t seq_num, uint8_t event,
										uint8_t use_int, uint32_t mode);

/* Software trigger of the given configured sequence.
 * Note : If the sequence was configured for triggering on an external event, it won't be anymore.
 */
void adc_trigger_sequence_conversion(uint8_t seq_num);


/***************************************************************************** */
/*   ADC Setup : private part : Clocks, Pins, Power and Mode   */
void adc_on(void (*adc_callback)(uint32_t));
void adc_off(void);


/***************************************************************************** */
/*                     Analog-to-Digital Converter                             */
/***************************************************************************** */
/* Analog-to-Digital Converter (ADC) */
struct lpc_adc
{
	volatile uint32_t ctrl;         /* 0x000 : A/D Control Register (R/W) */
	volatile uint32_t global_data;  /* 0x004 : A/D Global Data Register (R/W) */
	uint32_t reserved_0;
	volatile uint32_t int_en;  /* 0x00C : A/D Interrupt Enable Register (R/W) */
	volatile uint32_t data[8]; /* Offset: 0x010-0x02C A/D Channel 0..7 Data Register (R/W) */
	volatile const uint32_t status; /* 0x030 : A/D Status Register (R/ ) */
	volatile uint32_t trim;    /* 0x034 : A/D Trim Register (R/W) */
};
#define LPC_ADC_REGS      ((struct lpc_adc *) LPC_ADC_BASE)

/* ADC Control register bits */
#define LPC_ADC_CTRL_MASK  0x0F01FFFF
/* ADC_MCH_* are also used for interrupt register */
#define ADC_MCH_MASK    (0xFF << 0)
#define ADC_MCH(x)      (0x01 << ((x) & 0x07))
#define LPC_ADC_BURST   (0x01 << 16)

#define LPC_ADC_START_CONV_MASK (0x07 << 24)
#define LPC_ADC_START_CONV_NOW  (0x01 << 24)
#define LPC_ADC_START_CONV_EVENT(x) (((x) & 0x7) << 24)
#define LPC_ADC_START_EDGE_FALLING  (0x1 << 27)
#define LPC_ADC_START_EDGE_RISING   (0x0 << 27)
#define LPC_ADC_ADDITIONAL_MODE_MASK (0x1 << 27)

/* ADC Data register bits */
#define LPC_ADC_RESULT_SHIFT  6
#define LPC_ADC_RESULT_MASK   0x3FF
#define LPC_ADC_OVERRUN    (0x01 << 30)
#define LPC_ADC_CONV_DONE  (0x01 << 31)

/* For more readability when selecting a channel number */
#define LPC_ADC(x)    (x)


#endif /* DRIVERS_ADC_H */

