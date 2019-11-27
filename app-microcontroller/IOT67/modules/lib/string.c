/*
 *  linux/lib/string.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

/*
 * stupid library routines.. The optimized versions should generally be found
 * as inline code in <asm-xx/string.h>
 *
 * These are buggy as well..
 *
 * * Fri Jun 25 1999, Ingo Oeser <ioe@informatik.tu-chemnitz.de>
 * -  Added strsep() which will replace strtok() soon (because strsep() is
 *    reentrant and should be faster). Use only strsep() in new code, please.
 */

#include "lib/stddef.h"
#include "lib/stdint.h"

/**
 * memcpy - Copy one area of memory to another
 * @dest: Where to copy to
 * @src: Where to copy from
 * @count: The size of the area.
 */
void* memcpy(void* dest, const void* src, size_t count)
{
	unsigned long* dl = (unsigned long*)dest, *sl = (unsigned long*)src;
	char *d8, *s8;

	if (src == dest)
		return dest;

	/* while all data is aligned (common case), copy a word at a time */
	if ( (((uint32_t)dest | (uint32_t)src) & (sizeof(*dl) - 1)) == 0) {
		while (count >= sizeof(*dl)) {
			*dl++ = *sl++;
			count -= sizeof(*dl);
		}
	}
	/* copy the rest one byte at a time */
	d8 = (char *)dl;
	s8 = (char *)sl;
	while (count--) {
		*d8++ = *s8++;
	}
	return dest;
}

/**
 * memset - Fill a region of memory with the given value
 * @s: Pointer to the start of the area.
 * @c: The byte to fill the area with
 * @count: The size of the area.
 */
void* memset(void* s, int c, size_t count)
{
	unsigned long* sl = (unsigned long*) s;
	unsigned long cl = 0;
	char* s8;
	size_t i;

	/* do it one word at a time (32 bits or 64 bits) while possible */
	if ( ((uint32_t)s & (sizeof(*sl) - 1)) == 0) {
		for (i = 0; i < sizeof(*sl); i++) {
			cl <<= 8;
			cl |= c & 0xff;
		}
		while (count >= sizeof(*sl)) {
			*sl++ = cl;
			count -= sizeof(*sl);
		}
	}
	/* fill 8 bits at a time */
	s8 = (char*)sl;
	while (count--) {
		*s8++ = c;
	}
	return s;
}

/**
 * strcpy - Copy a %NUL terminated string
 * @dest: Where to copy the string to
 * @src: Where to copy the string from
 */
char* strcpy(char* dest, const char* src)
{
	char* tmp = dest;

	while ((*dest++ = *src++) != '\0') {
		/* nothing */;
	}
	return tmp;
}

/**
 * strncpy - Copy a length-limited, %NUL-terminated string
 * @dest: Where to copy the string to
 * @src: Where to copy the string from
 * @count: The maximum number of bytes to copy
 *
 * Note that unlike userspace strncpy, this does not %NUL-pad the buffer.
 * However, the result is not %NUL-terminated if the source exceeds
 * @count bytes.
 */
char* strncpy(char* dest, const char* src, size_t count)
{
	char* tmp = dest;

	while (count-- && (*dest++ = *src++) != '\0') {
		/* nothing */;
	}
	return tmp;
}

/**
 * strcmp - Compare two strings
 * @cs: One string
 * @ct: Another string
 */
int strcmp(const char* cs, const char* ct)
{
	register signed char __res;

	while (1) {
		if ((__res = *cs - *ct++) != 0 || !*cs++) {
			break;
		}
	}
	return __res;
}

/**
 * strncmp - Compare two length-limited strings
 * @cs: One string
 * @ct: Another string
 * @count: The maximum number of bytes to compare
 */
int strncmp(const char* cs, const char* ct, size_t count)
{
	register signed char __res = 0;

	while (count) {
		if ((__res = *cs - *ct++) != 0 || !*cs++) {
			break;
		}
		count--;
	}

	return __res;
}

/**
 * strchr - Find the first occurrence of a character in a string
 * @s: The string to be searched
 * @c: The character to search for
 */
char* strchr(const char* s, int c)
{
	for(; *s != (char) c; ++s) {
		if (*s == '\0') {
			return NULL;
		}
	}
	return (char *) s;
}

/**
 * strlen - Find the length of a string
 * @s: The string to be sized
 */
size_t strlen(const char* s)
{
	const char* sc;

	for (sc = s; *sc != '\0'; ++sc) {
		/* nothing */;
	}
	return sc - s;
}

/**
 * strrchr - Find the last occurrence of a character in a string
 * @s: The string to be searched
 * @c: The character to search for
 */
char* strrchr(const char* s, int c)
{
	const char* p = s + strlen(s);
	do {
		if (*p == (char)c) {
			return (char *)p;
		}
	} while (--p >= s);
	return NULL;
}

/**
 * strnlen - Find the length of a length-limited string
 * @s: The string to be sized
 * @count: The maximum number of bytes to search
 */
size_t strnlen(const char* s, size_t count)
{
	const char* sc;

	for (sc = s; count-- && *sc != '\0'; ++sc) {
		/* nothing */;
	}
	return sc - s;
}
