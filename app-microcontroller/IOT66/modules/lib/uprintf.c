/****************************************************************************
 *  lib/uprintf.c
 *
 * UART printf
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

#include <stdarg.h>
#include "lib/stdint.h"
#include "drivers/serial.h"
#include "lib/string.h"
#include "lib/stdio.h"


int uprintf(int uart_num, const char* format, ...)
{
	char printf_buf[SERIAL_OUT_BUFF_SIZE];
	va_list args;
	int r;

	va_start(args, format);
    r = vsnprintf(printf_buf, SERIAL_OUT_BUFF_SIZE, format, args);
    va_end(args);

	serial_write(uart_num, printf_buf, r);

    return r;
}


