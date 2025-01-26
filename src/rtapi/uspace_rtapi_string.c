//    Copyright 2005-2014, various authors
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

//
// This file contains some handy string parsing functions lifted from Linux
// 2.6.27.44, from the files mm/util.c and lib/argv_split.c.
//
// It gets compiled into the hostmot2 driver if the kernel does not provide
// its own versions of these functions: argv_split(), argv_free(), and
// kstrndup().
//


#include <rtapi_ctype.h>
#include <rtapi_slab.h>
#include <rtapi_string.h>

/**
 * rtapi_kstrndup - allocate space for and copy an existing string
 * @s: the string to duplicate
 * @max: read at most @max chars from @s
 * @gfp: the GFP mask used in the rtapi_kmalloc() call when allocating memory
 */
char *rtapi_kstrndup(const char *s, size_t max, rtapi_gfp_t gfp)
{
	(void)gfp;
	size_t len;
	char *buf;

	if (!s)
		return NULL;

	len = strnlen(s, max);
	buf = rtapi_kmalloc(len+1, gfp);
	if (buf) {
		memcpy(buf, s, len);
		buf[len] = '\0';
	}
	return buf;
}


/*
 * Helper function for splitting a string into an argv-like array.
 */

static const char *skip_sep(const char *cp)
{
	while (*cp && isspace(*cp))
		cp++;

	return cp;
}

static const char *skip_arg(const char *cp)
{
	while (*cp && !isspace(*cp))
		cp++;

	return cp;
}

static int count_argc(const char *str)
{
	int count = 0;

	while (*str) {
		str = skip_sep(str);
		if (*str) {
			count++;
			str = skip_arg(str);
		}
	}

	return count;
}

/**
 * rtapi_argv_free - free an argv
 * @argv - the argument vector to be freed
 *
 * Frees an argv and the strings it points to.
 */
void rtapi_argv_free(char **argv)
{
	char **p;
	for (p = argv; *p; p++)
		rtapi_kfree(*p);

	rtapi_kfree(argv);
}

/**
 * rtapi_argv_split - split a string at whitespace, returning an argv
 * @gfp: the GFP mask used to allocate memory
 * @str: the string to be split
 * @argcp: returned argument count
 *
 * Returns an array of pointers to strings which are split out from
 * @str.  This is performed by strictly splitting on white-space; no
 * quote processing is performed.  Multiple whitespace characters are
 * considered to be a single argument separator.  The returned array
 * is always NULL-terminated.  Returns NULL on memory allocation
 * failure.
 */
char **rtapi_argv_split(rtapi_gfp_t gfp, const char *str, int *argcp)
{
	int argc = count_argc(str);
	char **argv = rtapi_kzalloc(sizeof(*argv) * (argc+1), gfp);
	char **argvp;

	if (argv == NULL)
		goto out;

	if (argcp)
		*argcp = argc;

	argvp = argv;

	while (*str) {
		str = skip_sep(str);

		if (*str) {
			const char *p = str;
			char *t;

			str = skip_arg(str);

			t = rtapi_kstrndup(p, str-p, gfp);
			if (t == NULL)
				goto fail;
			*argvp++ = t;
		}
	}
	*argvp = NULL;

  out:
	return argv;

  fail:
	rtapi_argv_free(argv);
	return NULL;
}

EXPORT_SYMBOL(rtapi_kstrndup);
EXPORT_SYMBOL(rtapi_argv_split);
EXPORT_SYMBOL(rtapi_argv_free);
