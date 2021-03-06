/****************************************************************************
 *  lib/stdio.h
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

#ifndef LIB_STDIO_H
#define LIB_STDIO_H

#include <stdarg.h>
#include "lib/stdint.h"
#include "lib/string.h"


int vsnprintf(char *buf, size_t size, const char *fmt, va_list args);


int snprintf(char* buf, size_t size, const char *format, ...);


int uprintf(int uart_num, const char *format, ...);


#endif /* LIB_STDIO_H */
