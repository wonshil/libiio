/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2014 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * */

#ifndef DEBUG_H
#define DEBUG_H

#include "iio.h"
#include "iio-config.h"

#include <stdio.h>

#define NoLog_L 0
#define Error_L 1
#define Warning_L 2
#define Info_L 3
#define Debug_L 4

/* -------------------- */

#ifdef WITH_COLOR_DEBUG
#ifndef COLOR_DEBUG
#define COLOR_DEBUG   "\e[0;32m"
#endif
#ifndef COLOR_WARNING
#define COLOR_WARNING "\e[01;35m"
#endif
#ifndef COLOR_ERROR
#define COLOR_ERROR   "\e[01;31m"
#endif

#define COLOR_END "\e[0m"
#endif

/* Many of these debug printf include a Flawfinder: ignore, this is because,
 * according to https://cwe.mitre.org/data/definitions/134.html which describes
 * functions that accepts a format string as an argument, but the format
 * string originates from an external source. All the IIO_DEBUG, IIO_INFO,
 * IIO_WARNING, and IIO_ERRRO functions are called internally from the
 * library, have fixed format strings and can not be modified externally.
 */
#if (LOG_LEVEL >= Debug_L)
# ifdef COLOR_DEBUG
#  define IIO_DEBUG(str, ...) \
    fprintf(stdout, COLOR_DEBUG "DEBUG: " str COLOR_END, ##__VA_ARGS__) /* Flawfinder: ignore */
# else
#  define IIO_DEBUG(...) \
    fprintf(stdout, "DEBUG: " __VA_ARGS__) /* Flawfinder: ignore */
# endif
#else
#define IIO_DEBUG(...) do { } while (0)
#endif

#if (LOG_LEVEL >= Info_L)
# ifdef COLOR_INFO
#  define IIO_INFO(str, ...) \
    fprintf(stdout, COLOR_INFO str COLOR_END, ##__VA_ARGS__) /* Flawfinder: ignore */
# else
#  define IIO_INFO(...) \
    fprintf(stdout, __VA_ARGS__) /* Flawfinder: ignore */
# endif
#else
#define IIO_INFO(...) do { } while (0)
#endif

#if (LOG_LEVEL >= Warning_L)
# ifdef COLOR_WARNING
#  define IIO_WARNING(str, ...) \
    fprintf(stderr, COLOR_WARNING "WARNING: " str COLOR_END, ##__VA_ARGS__) /* Flawfinder: ignore */
# else
#  define IIO_WARNING(...) \
    fprintf(stderr, "WARNING: " __VA_ARGS__) /* Flawfinder: ignore */
# endif
#else
#define IIO_WARNING(...) do { } while (0)
#endif

#if (LOG_LEVEL >= Error_L)
# ifdef COLOR_ERROR
#  define IIO_ERROR(str, ...) \
    fprintf(stderr, COLOR_ERROR "ERROR: " str COLOR_END, ##__VA_ARGS__) /* Flawfinder: ignore */
# else
#  define IIO_ERROR(...) \
    fprintf(stderr, "ERROR: " __VA_ARGS__) /* Flawfinder: ignore */
# endif
#else
#define IIO_ERROR(...) do { } while (0)
#endif

#if defined(__MINGW32__)
#   define __iio_printf __attribute__((__format__(ms_printf, 3, 4)))
#elif defined(__GNUC__)
#   define __iio_printf __attribute__((__format__(printf, 3, 4)))
#else
#   define __iio_printf
#endif

void __iio_printf iio_ctx_printf(struct iio_context *ctx, unsigned int level,
				 const char *fmt, ...);

#define ctx_err(ctx, fmt, ...) iio_ctx_printf((ctx), Error_L, (fmt), __VA_ARGS__)
#define ctx_warn(ctx, fmt, ...) iio_ctx_printf((ctx), Warning_L, (fmt), __VA_ARGS__)
#define ctx_info(ctx, fmt, ...) iio_ctx_printf((ctx), Info_L, (fmt), __VA_ARGS__)
#define ctx_dbg(ctx, fmt, ...) iio_ctx_printf((ctx), Debug_L, (fmt), __VA_ARGS__)

#define dev_err(dev, fmt, ...) ctx_err(iio_device_get_context(dev), fmt, __VA_ARGS__)
#define dev_warn(dev, fmt, ...) ctx_warn(iio_device_get_context(dev), fmt, __VA_ARGS__)
#define dev_info(dev, fmt, ...) ctx_info(iio_device_get_context(dev), fmt, __VA_ARGS__)
#define dev_dbg(dev, fmt, ...) ctx_dbg(iio_device_get_context(dev), fmt, __VA_ARGS__)

#define chn_err(chn, fmt, ...) dev_err(iio_channel_get_device(dev), fmt, __VA_ARGS__)
#define chn_warn(chn, fmt, ...) dev_warn(iio_channel_get_device(dev), fmt, __VA_ARGS__)
#define chn_info(chn, fmt, ...) dev_info(iio_channel_get_device(dev), fmt, __VA_ARGS__)
#define chn_dbg(chn, fmt, ...) dev_dbg(iio_channel_get_device(dev), fmt, __VA_ARGS__)

#define buf_err(buf, fmt, ...) dev_err(iio_buffer_get_device(dev), fmt, __VA_ARGS__)
#define buf_warn(buf, fmt, ...) dev_warn(iio_buffer_get_device(dev), fmt, __VA_ARGS__)
#define buf_info(buf, fmt, ...) dev_info(iio_buffer_get_device(dev), fmt, __VA_ARGS__)
#define buf_dbg(buf, fmt, ...) dev_dbg(iio_buffer_get_device(dev), fmt, __VA_ARGS__)

#endif
