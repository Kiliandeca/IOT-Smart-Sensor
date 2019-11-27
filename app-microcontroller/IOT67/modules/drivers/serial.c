/****************************************************************************
 *  drivers/serial.c
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
/*                UARTs                                                        */
/***************************************************************************** */
/* UART driver for the integrated UARTs of the LPC122x.
 * Refer to LPC122x documentation (UM10441.pdf) for more information.
 */

/* Both UARTs are available, UART numbers are in the range 0 - 1 */

#include "core/system.h"
#include "core/pio.h"
#include "lib/string.h"
#include "lib/utils.h"
#include "lib/errno.h"
#include "drivers/serial.h"

struct uart_device
{
	uint32_t num;
	struct lpc_uart* regs;
	uint32_t baudrate;
	uint32_t config;
	uint8_t capabilities;
	uint8_t current_mode;

	/* Output buffer */
	volatile char out_buff[SERIAL_OUT_BUFF_SIZE];
	volatile uint32_t sending; /* Actual sending position in out buffer */
	/* This lock only prevents multiple calls to serial_write() to execute simultaneously */
	volatile uint32_t out_lock;
	volatile uint32_t out_length; /* actual position to add in out buffer */

	/* Input */
	void (*rx_callback)(uint8_t); /* Possible RX callback */
};

#define NUM_UARTS 2
static struct uart_device uarts[NUM_UARTS] = {
	{
		.num = 0,
		.regs = (struct lpc_uart*)LPC_UART_0,
		.baudrate = 0,
		.config = (LPC_UART_8BIT | LPC_UART_NO_PAR | LPC_UART_1STOP),
		.out_buff = {0},
		.sending = 0,
		.out_lock = 0,
		.rx_callback = NULL,
		.capabilities = (SERIAL_CAP_UART | SERIAL_CAP_RS485),
		.current_mode = SERIAL_MODE_UART,
	},
	{
		.num = 1,
		.regs = (struct lpc_uart*)LPC_UART_1,
		.baudrate = 0,
		.config = (LPC_UART_8BIT | LPC_UART_NO_PAR | LPC_UART_1STOP),
		.out_buff = {0},
		.sending = 0,
		.out_lock = 0,
		.rx_callback = NULL,
		.capabilities = (SERIAL_CAP_UART | SERIAL_CAP_IRDA),
		.current_mode = SERIAL_MODE_UART,
	},
};

static void uart_check_rx(struct uart_device* uart, uint32_t intr)
{
	if ((intr & LPC_UART_INT_MASK) == LPC_UART_INT_RX) {
		uint8_t data = uart->regs->func.buffer;
		if (uart->rx_callback != NULL) {
			/* Call the Rx callback */
			uart->rx_callback(data);
		} else {
			/* Echo ? */
			if (!uart->sending) {
				uart->regs->func.buffer = data;
			}
		}
	}
	/* FIXME : handle RX erors */
}

static void uart_check_tx(struct uart_device* uart, uint32_t intr)
{
	/* We are currently sending, send next char */
	if ((intr & LPC_UART_INT_MASK) == LPC_UART_INT_TX) {
		if (uart->out_buff && uart->sending && (uart->out_length > uart->sending)) {
			uart->regs->func.buffer = uart->out_buff[uart->sending];
			uart->sending++;
		} else {
			uart->sending = 0;
		}
	}
}

/* Generic UART handler */
static void UART_Handler(struct uart_device* uart)
{
	uint32_t intr = uart->regs->func.intr_pending;

	uart_check_rx(uart, intr);
	uart_check_tx(uart, intr);
}


/* Handlers */
void UART_0_Handler(void)
{
	UART_Handler(&uarts[0]);
}
void UART_1_Handler(void)
{
	UART_Handler(&uarts[1]);
}


/* Start sending buffer content */
static void uart_start_sending(uint32_t uart_num)
{
	struct uart_device* uart = &uarts[uart_num];

	if (uart->out_buff == NULL)
		return;

	if (!uart->sending && (uart->out_length != 0)) {
		uart->sending++;
		uart->regs->func.buffer = uart->out_buff[0];
	}
}


/****************************************************************************** */
/*    Serial send byte - quick function with almost no tests.
 * If the uart is not sending, the byte is placed directly in the data buffer and
 * the call returns 0.
 * Else, the call returns -EBUSY and nothing is sent.
 */
int serial_send_quickbyte(uint32_t uart_num, uint8_t data)
{
	struct uart_device* uart = &uarts[uart_num];
	if (!uart->sending) {
		uart->regs->func.buffer = data;
		return 0;
	} else {
		return -EBUSY;
	}
}

/***************************************************************************** */
/*    Serial Write
 *
 * Try to send at most "length" characters from "buf" on the requested uart.
 * Returns a negative value on error, or number of characters copied into output buffer,
 * witch may be less than requested "length"
 * Possible errors: requested uart does not exists (-EINVAL) or unable to acquire uart
 * lock (-EBUSY).
 *
 * Warning for Real Time : This implementation will block if there's already a
 * transmission ongoing.
 */
int serial_write(uint32_t uart_num, const char *buf, uint32_t length)
{
	struct uart_device* uart = NULL;

	if (uart_num >= NUM_UARTS)
		return -EINVAL;

	uart = &uarts[uart_num];
	/* Lock acquire */
	if (sync_lock_test_and_set(&uart->out_lock, 1) == 1) {
		return -EBUSY;
	}

	/* If UART is sending wait for buffer empty */
	/* FIXME : be smart for OS, call scheduler or return */
	while (uart->sending != 0) {
		/* If interrupt are masked, check for tx ourselves */
		if (get_priority_mask() != 0) {
			uart_check_tx(uart, uart->regs->func.intr_pending);
		}
	}

	if (length > SERIAL_OUT_BUFF_SIZE) {
		length = SERIAL_OUT_BUFF_SIZE;
	}
	memcpy((char*)uart->out_buff, buf, length);
	uart->out_length = length;

	/* Turn output on */
	uart_start_sending(uart_num);

	/* Release the lock */
	sync_lock_release(&uart->out_lock);

	return length;
}


/***************************************************************************** */
/*    Serial Flush
 *
 * Wait until all characters have been sent
 * Returns -EINVAL on error, 0 on success.
 * Possible errors: requested uart does not exists or unable to acquire uart lock.
 *
 * Warning for Real Time : This implementation will block if there's already a
 * transmission ongoing.
 */
int serial_flush(uint32_t uart_num)
{
	struct uart_device* uart = NULL;

	if (uart_num >= NUM_UARTS)
		return -EINVAL;

	uart = &uarts[uart_num];

	/* Active wait for message to be sent. If interrupts are
	 * disabled, call the UART handler while waiting. */
	while (uart->sending) {
		if (get_priority_mask() != 0) {
			uart_check_tx(uart, uart->regs->func.intr_pending);
		}
	}

	return 0;
}


/***************************************************************************** */
/*   UART Setup : private part : Clocks, Pins, Power and Mode   */

struct uart_clk_cfg {
	uint32_t baudrate;
	uint8_t divisor_latch_lsb;
	uint8_t div_add_val;
	uint8_t mul_val;
};
static struct uart_clk_cfg uart_clk_table[] = {
	{ 250000, 12, 0, 1},
	{ 1152000, 2, 3, 10},
	{ 0, 0, 0, 0, },
};

/* UART Clock Setup */
/* Note : for both uarts we use full peripheral clock.
 *    With a minimum main clock of 12MHz, this gives 12MHz/16 = 750kbauds at least
 *    for UARTs baudrates.
 * Note : IRQ are off, whether called by system update or by UART on helper
 */
static void uart_clk_on(uint32_t uart_num, uint32_t baudrate)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	struct lpc_uart* uart = uarts[uart_num].regs; /* Get the right registers */
	uint32_t div = 0, pclk = 0;
	/* Save baudrate value */
	uarts[uart_num].baudrate = baudrate;
	/* Configure UART clock */
	pclk = get_main_clock(); /* See above note */
	div = (pclk / (baudrate * 16));
	sys_config->uart_clk_div[uart_num] = 0x01;
	/* The easy one : divider is an integer, or baudrate is low enough for the aproximation */
	if ((baudrate <= 115200) || ((div * baudrate * 16) == pclk)) {
		uart->line_ctrl |= LPC_UART_ENABLE_DLAB;
		uart->ctrl.divisor_latch_lsb = (div & 0xff);
		uart->ctrl.divisor_latch_msb = ((div >> 8) & 0xFF);
		uart->line_ctrl &= ~LPC_UART_ENABLE_DLAB;
	} else {
		int i = 0;
		/* Do not try to communicate at high speed with a low speed clock ... or compute the table
		 * for your clock rate */
		if (pclk != (48 * 1000 * 1000)) {
			return;
		}
		while (uart_clk_table[i].baudrate != 0) {
			if (uart_clk_table[i].baudrate < baudrate) {
				i++;
				continue;
			}
			uart->line_ctrl |= LPC_UART_ENABLE_DLAB;
			uart->ctrl.divisor_latch_lsb = uart_clk_table[i].divisor_latch_lsb;
			uart->ctrl.divisor_latch_msb = 0;
			uart->fractional_div = (uart_clk_table[i].div_add_val | (uart_clk_table[i].mul_val << 4));
			uart->line_ctrl &= ~LPC_UART_ENABLE_DLAB;
			break;
		}
	}
}
static void uart_clk_off(uint32_t uart_num)
{
	struct lpc_sys_config* sys_config = LPC_SYS_CONFIG;
	/* Erase saved baudrate */
	uarts[uart_num].baudrate = 0;
	sys_config->uart_clk_div[uart_num] = 0;
}

static uint32_t uart_setup(uint32_t uart_num)
{
	struct lpc_uart* uart = uarts[uart_num].regs; /* Get the right registers */
	uint32_t status = 0;
	/* Set up UART mode */
	uart->line_ctrl = uarts[uart_num].config;
	/* Clear all fifo, reset and enable them */
	/* Note : fifo trigger level is one byte */
	uart->ctrl.fifo_ctrl = (LPC_UART_FIFO_EN | LPC_UART_TX_CLR | LPC_UART_RX_CLR);
	/* Clear the Line Status Register, return it to prevent compiler from removing the read */
	status = uart->line_status;
	/* Enable UART Interrupt */
	uart->func.intr_enable = (LPC_UART_RX_INT_EN | LPC_UART_TX_INT_EN);

	return status;
}

/* Change UART configuration (number of data, parity and stop bits).
 * config is a mask of LPC_UART_xBIT (x = 5..8), LPC_UART_xSTOP (x = 1..2)
 *   and one of LPC_UART_NO_PAR, LPC_UART_ODD_PAR or LPC_UART_EVEN_PAR.
 */
int uart_set_config(uint8_t uart_num, uint32_t config)
{
	struct uart_device* uart = NULL;

	if (uart_num >= NUM_UARTS)
		return -EINVAL;
	uart = &uarts[uart_num];
	uart->config = config;
	return 0;
}

struct uart_def
{
	uint32_t irq;
	uint32_t power_offset;
};
static struct uart_def uart_defs[NUM_UARTS] = {
	{ UART0_IRQ, LPC_SYS_ABH_CLK_CTRL_UART0 },
	{ UART1_IRQ, LPC_SYS_ABH_CLK_CTRL_UART1 },
};

/***************************************************************************** */
/*   Public access to UART setup   */

/* Allow any change to the main clock to be forwarded to us */
void uart_clk_update(void)
{
	int i = 0;
	for (i = 0; i < NUM_UARTS; i++) {
		if (uarts[i].baudrate != 0) {
			uart_clk_on(i, uarts[i].baudrate);
		}
	}
}

/* Change uart mode to RS485
 * return -ENODEV when the device does not support RS485 mode.
 */
int uart_set_mode_rs485(uint32_t uart_num, uint32_t control, uint8_t addr, uint8_t delay)
{
	struct uart_device* uart = NULL;
	struct lpc_uart* uart_regs = NULL;
	if (uart_num >= NUM_UARTS)
		return -EINVAL;
	uart = &uarts[uart_num];
	uart_regs = uart->regs; /* Get the right registers */

	if (!(uart->capabilities & SERIAL_CAP_RS485)) {
		return -ENODEV;
	}
	/* Disable all other modes */
	uart_regs->irda_ctrl = 0;

	/* Set current mode */
	uart->current_mode = SERIAL_MODE_RS485;
	uart_regs->RS485_ctrl = (control & 0xFF);
	uart_regs->RS485_addr_match = LPC_RS485_ADDR(addr);
	uart_regs->RS485_dir_ctrl_delay = LPC_RS485_DIR_DELAY(delay);
	return 0;
}

/* Change uart mode to IRDA
 * pulse_width is the number of clock cycles for a pulse. Should dbe a power of 2.
 * return -ENODEV when the device does not support IrDA mode.
 */
int uart_set_mode_irda(uint32_t uart_num, uint32_t control, uint16_t pulse_width)
{
	struct uart_device* uart = NULL;
	struct lpc_uart* uart_regs = NULL;
	uint8_t pulse_div = 0;

	if (uart_num >= NUM_UARTS)
		return -EINVAL;
	uart = &uarts[uart_num];
	uart_regs = uart->regs; /* Get the right registers */

	if (!(uart->capabilities & SERIAL_CAP_IRDA)) {
		return -ENODEV;
	}
	/* Disable all other modes */
	uart_regs->RS485_ctrl = 0;

	/* Set current mode */
	uart->current_mode = SERIAL_MODE_IRDA;
	/* Setup IrDA */
	if (pulse_width < 2) {
		pulse_div = 0;
	} else {
		pulse_div = (31 - clz(pulse_width & 0x1FF));
	}
	uart_regs->irda_ctrl = control | LPC_IRDA_PULSEDIV(pulse_div);
	return 0;
}


/* Do we need to allow setting of other parameters ? (Other than 8n1) */
int uart_on(uint32_t uart_num, uint32_t baudrate, void (*rx_callback)(uint8_t))
{
	struct uart_def* uart = NULL;
	uint32_t status = 0;

	if (uart_num >= NUM_UARTS)
		return -EINVAL;
	uart = &uart_defs[uart_num];
	uarts[uart_num].rx_callback = rx_callback;

	NVIC_DisableIRQ( uart->irq );
	/* Turn On power */
	subsystem_power(uart->power_offset, 1);
	/* Setup clock acording to baudrate */
	uart_clk_on(uart_num, baudrate);
	/* Setup mode, fifo, ... */
	status = uart_setup(uart_num);
	/* Enable interrupts back on */
	NVIC_EnableIRQ( uart->irq );

	return status;
}

void uart_off(uint32_t uart_num)
{
	struct uart_def* uart = NULL;
	if (uart_num >= NUM_UARTS)
		return;
	uart = &uart_defs[uart_num];

	NVIC_DisableIRQ( uart->irq );
	uart_clk_off(uart_num);
	/* Turn Off power */
	subsystem_power(uart->power_offset, 0);
	uarts[uart_num].rx_callback = NULL;
}



