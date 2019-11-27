/****************************************************************************
 *   core/bootstrap.c
 *
 * This file holds the bootstrap code, the vector table, and nothing more.
 * The bootstrap code is the code of the reset handler, which is called by
 *   the bootloader (executed from internal ROM after power on or reset).
 * The reset handler code perform copy of data section, clears bss, and
 *   calls the main function.
 *
 *
 * Copyright 2012 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 * Example code from frozeneskimo.com :
 *   http://dev.frozeneskimo.com/notes/getting_started_with_cortex_m3_cmsis_and_gnu_tools
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
 *****************************************************************************/


extern unsigned int _end_stack;
extern unsigned int _end_text;
extern unsigned int _start_data;
extern unsigned int _end_data;
extern unsigned int _start_bss;
extern unsigned int _end_bss;

extern int main(void);

/* Cortex M0 core interrupt handlers */
void Reset_Handler(void);
void NMI_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void HardFault_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void SVC_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void SysTick_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
/* LPC12xx specific interrupt handlers */
void WAKEUP_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void I2C_0_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void TIMER_0_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void TIMER_1_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void TIMER_2_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void TIMER_3_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void SSP_0_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void UART_0_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void UART_1_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void Comparator_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void ADC_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void WDT_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void BOD_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void PIO_0_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void PIO_1_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void PIO_2_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void DMA_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void RTC_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));



void Dummy_Handler(void);


/***************************************************************************** */
/*                      Vector table                                           */
/***************************************************************************** */
void *vector_table[] __attribute__ ((section(".vectors"))) = {
	&_end_stack, /* Initial SP value */ /* 0 */
	Reset_Handler,
	NMI_Handler,
	HardFault_Handler,
	0,
	0, /* 5 */
	0,
	/* Entry 7 (8th entry) must contain the 2’s complement of the check-sum
	   of table entries 0 through 6. This causes the checksum of the first 8
	   table entries to be 0 */
	(void *)0xDEADBEEF, /* Actually, this is done using an external tool. */
	0,
	0,
	0, /* 10 */
	SVC_Handler,
	0,
	0,
	PendSV_Handler,
	SysTick_Handler, /* 15 */
	/* LPC12xx specific interrupt vectors, see chapter 3 of LPC12xx User manual (UM10441) */
	WAKEUP_Handler,     /* 16 */ /* IRQ0 */
	WAKEUP_Handler,
	WAKEUP_Handler,
	WAKEUP_Handler,
	WAKEUP_Handler, /* 20 */
	WAKEUP_Handler, /* 21 */ /* IRQ5 */
	WAKEUP_Handler,
	WAKEUP_Handler,
	WAKEUP_Handler,
	WAKEUP_Handler, /* 25 */
	WAKEUP_Handler, /* 26 */ /* IRQ10 */
	WAKEUP_Handler,
	I2C_0_Handler,
	TIMER_0_Handler, /* CT16B0 */
	TIMER_1_Handler, /* CT16B1 */ /* 30 */ 
	TIMER_2_Handler, /* CT32B0 */ /* IRQ15 */
	TIMER_3_Handler, /* CT32B1 */
	SSP_0_Handler,
	UART_0_Handler,
	UART_1_Handler, /* 35 */
	Comparator_Handler, /* IRQ20 */
	ADC_Handler,
	WDT_Handler,
	BOD_Handler,
	0,              /* 40 */
	PIO_0_Handler,  /* IRQ25 */
	PIO_1_Handler,
	PIO_2_Handler,
	0,
	DMA_Handler,   /* 45 */
	RTC_Handler,   /* IRQ30 */
	0,
};


extern void rom_helpers_init(void);
/*
 * This is the entry point of the programm
 * It does the set up of the memory and then starts the main.
 */
void Reset_Handler(void) {
	unsigned int *src, *dst;

	/* Copy data section from flash to RAM */
	src = &_end_text;
	dst = &_start_data;
	while (dst < &_end_data)
		*dst++ = *src++;

	/* Clear the bss section */
	dst = &_start_bss;
	while (dst < &_end_bss)
		*dst++ = 0;

	/* Initialize rom based division helpers */
	rom_helpers_init();
	/* Start main programm */
	main();
}

void Dummy_Handler(void) {
	while (1);
}

