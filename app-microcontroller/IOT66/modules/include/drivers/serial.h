/****************************************************************************
 *  drivers/serial.h
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

#ifndef DRIVERS_SERIAL_H
#define DRIVERS_SERIAL_H


#include "lib/stdint.h"
#include "core/lpc_regs.h"


#define UART0  0
#define UART1  1
#define UART2  2
#define UART3  3


#define SERIAL_CAP_UART   (1 << 0)
#define SERIAL_CAP_RS485  (1 << 1)
#define SERIAL_CAP_IRDA   (1 << 2)

#define SERIAL_MODE_UART  SERIAL_CAP_UART
#define SERIAL_MODE_RS485 SERIAL_CAP_RS485
#define SERIAL_MODE_IRDA  SERIAL_CAP_IRDA


/* This buffer size is the maximum write size for a serial_printf or a uprintf call.
 * Previously this value was 64, but it is often too short, so I set it to 96, which
 *    should be OK for most cases. In need of a bigger write buffer, change this value
 *    or perform multiple write calls (better).
 */
#define SERIAL_OUT_BUFF_SIZE 96


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
int serial_write(uint32_t uart_num, const char *buf, uint32_t length);

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
int serial_flush(uint32_t uart_num);


/****************************************************************************** */
/*    Serial send byte - quick function with almost no tests.
 * If the uart is not sending, the byte is placed directly in the data buffer and
 * the call returns 0.
 * Else, the call returns -EBUSY and nothing is sent.
 */
int serial_send_quickbyte(uint32_t uart_num, uint8_t data);


/***************************************************************************** */
/*   Public access to UART setup   */


/* Change UART configuration (number of data, parity and stop bits).
 * config is a mask of LPC_UART_xBIT (x = 5..8), LPC_UART_xSTOP (x = 1..2)
 *   and one of LPC_UART_NO_PAR, LPC_UART_ODD_PAR or LPC_UART_EVEN_PAR.
 */
int uart_set_config(uint8_t uart_num, uint32_t config);

/* Change uart mode to RS485
 * return -ENODEV when the device does not support RS485 mode.
 */
int uart_set_mode_rs485(uint32_t uart_num, uint32_t control, uint8_t addr, uint8_t delay);

/* Change uart mode to IRDA
 * return -ENODEV when the device does not support IrDA mode.
 */
int uart_set_mode_irda(uint32_t uart_num, uint32_t control, uint16_t pulse_width);

/* Allow any change to the main clock to be forwarded to us */
void uart_clk_update(void);

/* Do we need to allow setting of other parameters ? (Other than 8n1) */
int uart_on(uint32_t uart_num, uint32_t baudrate, void (*rx_callback)(uint8_t));

void uart_off(uint32_t uart_num);



/***************************************************************************** */
/*                     Universal Asynchronous Receiver Transmitter             */
/***************************************************************************** */
/* Universal Asynchronous Receiver Transmitter (UART) */
struct lpc_uart_func {
	volatile uint32_t buffer; /* 0x000 : Transmit / Receiver Buffer Register (R/W) */
	volatile uint32_t intr_enable; /* 0x004 : Interrupt Enable Register (R/W) */
	volatile uint32_t intr_pending; /* 0x008 : Interrupt ID Register (R/-) */
};
struct lpc_uart_ctrl {
	volatile uint32_t divisor_latch_lsb;  /* 0x000 : Divisor Latch LSB (R/W) */
	volatile uint32_t divisor_latch_msb;  /* 0x004 : Divisor Latch MSB (R/W) */
	volatile uint32_t fifo_ctrl;  /* 0x008 : Fifo Control Register (-/W) */
};
struct lpc_uart
{
	union {
		struct lpc_uart_func func;
		struct lpc_uart_ctrl ctrl;
	};
	volatile uint32_t line_ctrl;   /* 0x00C : Line Control Register (R/W) */
	volatile uint32_t modem_ctrl;  /* 0x010 : Modem control Register (R/W) */
	volatile const uint32_t line_status;   /* 0x014 : Line Status Register (R/ ) */
	volatile const uint32_t modem_status;  /* 0x018 : Modem status Register (R/ ) */
	volatile uint32_t scratch_pad;  /* 0x01C : Scratch Pad Register (R/W) */
	volatile uint32_t auto_baud_ctrl;  /* 0x020 : Auto-baud Control Register (R/W) */
	volatile uint32_t irda_ctrl;       /* 0x024 : UART IrDA Control Register (R/W) */
	volatile uint32_t fractional_div;  /* 0x028 : Fractional Divider Register (R/W) */
	uint32_t reserved_1;
	volatile uint32_t transmit_enable;  /* 0x030 : Transmit Enable Register (R/W) */
	uint32_t reserved_2[6];
	volatile uint32_t RS485_ctrl;       /* 0x04C : RS-485/EIA-485 Control Register (R/W) */
	volatile uint32_t RS485_addr_match; /* 0x050 : RS-485/EIA-485 address match Register (R/W) */
	volatile uint32_t RS485_dir_ctrl_delay;  /* 0x054 : RS-485/EIA-485 direction control delay Register (R/W) */
	volatile uint32_t fifo_level;  /* 0x058 : Fifo Level Register (R/-) */
};
#define LPC_UART_0        ((struct lpc_uart *) LPC_UART0_BASE)
#define LPC_UART_1        ((struct lpc_uart *) LPC_UART1_BASE)

/* Line Control Register */
#define LPC_UART_5BIT          (0x00 << 0)
#define LPC_UART_6BIT          (0x01 << 0)
#define LPC_UART_7BIT          (0x02 << 0)
#define LPC_UART_8BIT          (0x03 << 0)
#define LPC_UART_1STOP         (0x00 << 2)
#define LPC_UART_2STOP         (0x01 << 2)
#define LPC_UART_NO_PAR        (0x00 << 3)
#define LPC_UART_ODD_PAR      ((0x01 << 3) | (0x00 << 4))
#define LPC_UART_EVEN_PAR      ((0x01 << 3) | (0x01 << 4))
#define LPC_UART_ENABLE_DLAB   (0x01 << 7)
/* FIFO Control Register */
#define LPC_UART_FIFO_EN       (0x01 << 0)
#define LPC_UART_RX_CLR        (0x01 << 1)
#define LPC_UART_TX_CLR        (0x01 << 2)
#define LPC_UART_DMA_MODE_EN   (0x01 << 3)
#define LPC_UART_FIFO_TRIG(x)  ((x & 0x03) << 6) /* 1 / 4 / 8 / 14 chars */
/* Interrupt Enable Register */
#define LPC_UART_RX_INT_EN     (0x01 << 0)
#define LPC_UART_TX_INT_EN     (0x01 << 1)
#define LPC_UART_RX_STATUS_INT_EN   (0x01 << 2)
/* Interrupt status */
#define LPC_UART_INT_MASK      (0x7 << 1)
#define LPC_UART_INT_MODEM     (0x0 << 1)
#define LPC_UART_INT_TX        (0x1 << 1)
#define LPC_UART_INT_RX        (0x2 << 1)
#define LPC_UART_INT_RX_STATUS (0x3 << 1)
#define LPC_UART_INT_TIMEOUT   (0x6 << 1)
/* RS485 Control */
#define LPC_RS485_ENABLE       (0x1 << 0)
#define LPC_RS485_RX_DIS       (0x1 << 1)
#define LPC_RS485_AUTO_ADDR_EN (0x1 << 2)
#define LPC_RS485_DIR_PIN_RTS  (0x0 << 3)
#define LPC_RS485_DIR_PIN_DTR  (0x1 << 3)
#define LPC_RS485_AUTO_DIR_EN  (0x1 << 4)
#define LPC_RS485_DIR_CTRL_INV (0x1 << 5)
/* RS485 */
#define LPC_RS485_ADDR(x)  ((x) & 0xFF)
#define LPC_RS485_DIR_DELAY(x)  ((x) & 0xFF)
/* IrDA */
#define LPC_IRDA_PULSEDIV(x)  (((x) & 0x07) << 3)

#endif /* DRIVERS_SERIAL_H */
