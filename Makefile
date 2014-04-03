# libiio - Library for interfacing industrial I/O (IIO) devices
#
# Copyright (C) 2014 Analog Devices, Inc.
# Author: Paul Cercueil <paul.cercueil@analog.com>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

PREFIX ?= /usr/local

VERSION_MAJOR = 0
VERSION_MINOR = 1

LIBNAME = libiio.so
SONAME = $(LIBNAME).$(VERSION_MAJOR)
LIBIIO = $(SONAME).$(VERSION_MINOR)

COMPILER ?= gcc
CC := $(CROSS_COMPILE)$(COMPILER)
ANALYZER := clang --analyze
INSTALL ?= install

# XXX: xml2-config is not sysroot aware...
SYSROOT := $(shell $(CC) --print-sysroot)
XML2_CFLAGS := $(shell $(SYSROOT)/usr/bin/xml2-config --cflags)
XML2_LIBS := $(shell $(SYSROOT)/usr/bin/xml2-config --libs)

CFLAGS := $(XML2_CFLAGS) -Wall -fPIC -fvisibility=hidden \
	-ansi -D_POSIX_C_SOURCE=200809L
LDFLAGS := $(XML2_LIBS)

ifdef DEBUG
	CFLAGS += -g -ggdb
	CPPFLAGS += -DLOG_LEVEL=4 #-DWITH_COLOR_DEBUG
else
	CFLAGS += -O2
endif

ifdef V
	CMD:=
	SUM:=@\#
else
	CMD:=@
	SUM:=@echo
endif

OBJS := context.o device.o channel.o local.o xml.o network.o buffer.o

.PHONY: all clean tests examples analyze install install-lib uninstall uninstall-lib

$(LIBIIO): $(OBJS)
	$(SUM) "  LD      $@"
	$(CMD)$(CC) -shared -Wl,-soname,$(SONAME) -o $@ $^ $(LDFLAGS) $(CFLAGS)

all: $(LIBIIO) iiod tests examples

clean-tests clean-examples clean-iiod:
	$(CMD)$(MAKE) -C $(@:clean-%=%) clean

clean: clean-tests clean-iiod
	$(SUM) "  CLEAN   ."
	$(CMD)rm -f $(LIBIIO) $(OBJS) $(OBJS:%.o=%.plist)

analyze:
	$(ANALYZER) $(CFLAGS) $(OBJS:%.o=%.c)

tests examples iiod: $(LIBIIO)
	$(CMD)$(MAKE) -C $@

%.o: %.c
	$(SUM) "  CC      $@"
	$(CMD)$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

install-tests install-examples install-iiod:
	$(CMD)$(MAKE) -C $(@:install-%=%) install

uninstall-tests uninstall-examples uninstall-iiod:
	$(CMD)$(MAKE) -C $(@:uninstall-%=%) uninstall

install-lib: $(LIBIIO)
	$(INSTALL) -D $(LIBIIO) $(DESTDIR)$(PREFIX)/lib/$(LIBIIO)
	ln -sf $(LIBIIO) $(DESTDIR)$(PREFIX)/lib/$(SONAME)

install: install-lib install-iiod install-tests
	$(INSTALL) -D -m 0644 iio.h $(DESTDIR)$(PREFIX)/include/iio.h
	ln -sf $(SONAME) $(DESTDIR)$(PREFIX)/lib/$(LIBNAME)

uninstall-lib:
	rm -f $(DESTDIR)$(PREFIX)/lib/$(LIBIIO) $(DESTDIR)$(PREFIX)/lib/$(SONAME)

uninstall: uninstall-lib uninstall-iiod uninstall-tests
	rm -f $(DESTDIR)$(PREFIX)/include/iio.h $(DESTDIR)$(PREFIX)/lib/$(LIBNAME)
