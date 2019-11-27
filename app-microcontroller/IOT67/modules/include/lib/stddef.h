/****************************************************************************
 *   lib/stddef.h
 *
 * Copyright 2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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

#ifndef LIB_STDDEF_H
#define LIB_STDDEF_H


#undef __SIZE_TYPE__
#define __SIZE_TYPE__ long unsigned int
typedef __SIZE_TYPE__ size_t;

#define NULL ((void *)0)


/**
 * offsetof - return the offset of a member in the containing structure.
 */
#ifdef __builtin_offsetof
 #define offsetof(type, member)  __builtin_offsetof (type, member)
#else
 #define offsetof(type, member) ((size_t) &((type *)0)->member)
#endif

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:        the pointer to the member.
 * @type:       the type of the container struct this is embedded in.
 * @member:     the name of the member within the struct.
 */
#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})


#endif /* LIB_STDDEF_H */
