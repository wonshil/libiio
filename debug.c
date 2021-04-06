// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2014 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#include "debug.h"
#include "iio-private.h"

#include <stdarg.h>

#define __printf(a, b) __attribute__((__format__(printf, a, b)))

void __iio_printf iio_ctx_printf(struct iio_context *ctx, unsigned int level,
				 const char *fmt, ...)
{
	va_list ap;
	FILE *out;

	va_start(ap, fmt);

	if (level <= ctx->log_level) {
		out = level <= ctx->max_level_stderr ? stderr : stdout;

		vfprintf(out, fmt, ap);
	}

	va_end(ap);
}
