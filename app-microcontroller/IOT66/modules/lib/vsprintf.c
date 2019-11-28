/****************************************************************************
 *  lib/vsprintf.c
 *
 * Code based on lib/vsprintf.c from linux kernel.
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
#include "lib/string.h"

#define ZEROPAD   (1 <<  0)  /* pad with zero */
#define SIGNED    (1 <<  1)  /* unsigned/signed long */
#define SIGN      (1 <<  2)  /* show plus */
#define SPACE     (1 <<  3)  /* space if plus */
#define LEFT      (1 <<  4)  /* left justified */
#define LOWERCASE (1 <<  5)  /* use lowercase in hex (must be 32 == 0x20) */
#define SPECIAL   (1 <<  6)  /* prefix hex with "0x", octal with "0" */
#define HEXA      (1 <<  7)  /* output hexa-decimal */

/* In our case, anything that does not fit in 20 chars does not fit in an unsigned int. */
#define TMP_NUM_BUF_SIZE 20
static int convert(char* buf, char* end, uint32_t flags, uint32_t width, uint32_t num)
{
	static const char digits[16] = "0123456789ABCDEF";
	char tmp[TMP_NUM_BUF_SIZE];
	int i = 0, length = 0;
	char sign = 0;

	if (width > TMP_NUM_BUF_SIZE) {
		width = TMP_NUM_BUF_SIZE;
	}

	/* Store sign, and convert to unsigned */
	if (flags & SIGNED) {
		if (width) {
			width--;
		}
		if ((signed long)num < 0) {
			sign = '-';
			num = -(signed long)num;
		}
	} /* Do we need to remove 2 to width in case of "SPECIAL" flag ? */

	/* Generate full string in tmp[], in reverse order */
	if (num == 0) {
		tmp[i++] = '0';
	} else if (flags & HEXA) {
		/* low_case = 0 or 0x20. ORing digits or letters with 'low_case'
		 * produces same digits or (maybe lowercased) letters */
		uint32_t low_case = (flags & LOWERCASE) ? 0x20 : 0;
		do {
			tmp[i++] = (digits[num & 0x0F] | low_case);
			num = (num >> 4);
		} while (num);
	} else {
		while (num) {
			tmp[i++] = (num % 10) + '0';
			num = num / 10;
		}
	}

	/* Add sign, pad if reqiered */
	if (flags & ZEROPAD) {
		while (i < width) {
			tmp[i++] = '0';
		}
	}
	if (sign) {
		tmp[i++] = sign;
	} else if (flags & SIGN) {
		tmp[i++] = '+';
	} else if (flags & SPACE) {
		tmp[i++] = ' ';
	} else if (flags & SPECIAL) {
		tmp[i++] = 'x';
		tmp[i++] = '0';
	}
	while (i < width) {
		tmp[i++] = ' ';
	}

	/* And reverse string */
	length = i;
	while (i && (buf < end)) {
		i--;
		*(buf++) = tmp[i];
	}
	if (i)
		return -i;

	return length;
}

int vsnprintf(char *buf, size_t size, const char *fmt, va_list args)
{
    char* start = buf;
    char* end = buf + size - 1; /* leave one char for terminating null byte */

	/* Parse format string */
	while ((buf < end) && *fmt) {
		uint32_t flags = 0;
		uint32_t width = 0;

		if (*fmt != '%') {
			*buf++ = *fmt++;
			continue;
		}

		/* Do we have a conversion specifier ? */
		fmt++;
		if (*fmt == '%') {
			*buf++ = *fmt++;
			continue;
		}
		/* We got a conversion specifier, any modifier ? */
		/* Note that '-' won't be handled */
		while (1) {
			int found = 1;
			switch (*fmt) {
				case '-': flags |= LEFT;    break;
				case '+': flags |= SIGN;    break;
				case ' ': flags |= SPACE;   break;
				case '#': flags |= SPECIAL; break;
				case '0': flags |= ZEROPAD; break;
				default:  found = 0;
			}
			if (!found)
				break;
			fmt++;
		}
		/* Field width ? */
		while ((*fmt >= '0') && (*fmt <= '9')) {
			width = (width * 10) + (*fmt - '0');
			fmt++;
		}
		/* We do not handle floats, floats have nothing to do with embeded systems */
		/* Skip any precision */
		if (*fmt == '.') {
			do {
				fmt++;
			} while ((*fmt >= '0') && (*fmt <= '9'));
		}
		/* Qualifier ? Should we really handle length modifiers ?
		 * The while loop is here to skip these. */
		/* Handle conversion, if supported. Note that we may spend some time in here
         * while waiting for the buffer to get available again  */
		while (*fmt) {
			int found = 1;
			switch (*fmt) {
				/* signed */
				case 'd':
				case 'i':
					flags |= SIGNED;
					buf += convert(buf, end, flags, width, (uint32_t)va_arg(args, signed int));
					break;
				/* unsigned */
				case 'x':
					flags |= LOWERCASE;
				case 'X':
					flags |= HEXA;
				case 'u':
					buf += convert(buf, end, flags, width, (uint32_t)va_arg(args, unsigned int));
					break;
				/* string */
				case 's': {
					/* Copy string to buf */
					char* tmp = va_arg(args, char *);
					while ((buf < end) && *tmp) {
						*buf++ = *tmp++;
					}
					break;
				}
				/* character */
				case 'c':
					*buf++ = (char)va_arg(args, unsigned int);
					break;
				default:
					found = 0;
			}
			fmt++;
			if (found)
				break;
		}
	}
	*buf = '\0';
	return (buf - start);
}


int snprintf(char* buf, size_t size, const char *format, ...)
{
	va_list args;
	int r;

	va_start(args, format);
    r = vsnprintf(buf, size, format, args);
    va_end(args);

    return r;
}


