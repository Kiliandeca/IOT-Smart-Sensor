/****************************************************************************
 *   lib/stdint.h
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 r
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

#ifndef LIB_STDINT_H
#define LIB_STDINT_H

/* Signed */
typedef signed char int8_t;
typedef short int int16_t;
typedef int int32_t;
__extension__
typedef long long int int64_t;
/* Small types - Signed */
typedef signed char int_least8_t;
typedef short int int_least16_t;
typedef int int_least32_t;
__extension__
typedef long long int int_least64_t;
/* Fast types - Signed */
typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef int int_fast32_t;
__extension__
typedef long long int int_fast64_t;


/* Unsigned */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
__extension__
typedef unsigned long long int uint64_t;
/* Small types - Unsigned */
typedef unsigned char uint_least8_t;
typedef unsigned short int uint_least16_t;
typedef unsigned int uint_least32_t;
__extension__
typedef unsigned long long int uint_least64_t;
/* Fast types - Unsigned */
typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
__extension__
typedef unsigned long long int uint_fast64_t;

typedef long unsigned int size_t;


/* Types for `void *' pointers.  */
typedef int intptr_t;
typedef unsigned int uintptr_t;


/* Largest integral types.  */
__extension__
typedef long long int intmax_t;
__extension__
typedef unsigned long long int uintmax_t;

#  define __INT64_C(c)  c ## LL
#  define __UINT64_C(c) c ## ULL


/* Limits of integral types.  */

/* Minimum of signed integral types.  */
#define INT8_MIN       (-128)
#define INT16_MIN      (-32767-1)
#define INT32_MIN      (-2147483647-1)
#define INT64_MIN      (-__INT64_C(9223372036854775807)-1)
/* Maximum of signed integral types.  */
#define INT8_MAX       (127)
#define INT16_MAX      (32767)
#define INT32_MAX      (2147483647)
#define INT64_MAX      (__INT64_C(9223372036854775807))

/* Maximum of unsigned integral types.  */
#define UINT8_MAX      (255)
#define UINT16_MAX     (65535)
#define UINT32_MAX     (4294967295U)
#define UINT64_MAX     (__UINT64_C(18446744073709551615))


#define SIZE_MAX     (4294967295UL)

#define __WORDSIZE  32


#endif /* LIB_STDINT_H */
