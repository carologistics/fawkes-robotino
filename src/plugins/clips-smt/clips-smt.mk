#*****************************************************************************
#            Makefile Build System for Fawkes: clips-smt Plugin
#                            -------------------
#   Created on Fri Jan 20 16:34 2017
#   Copyright (C) Igor Nicolai Bongartz AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

include $(BUILDSYSDIR)/boost.mk

# test z3
ifneq ($(wildcard $(SYSROOT)/usr/local/include/z3++.h),)
  HAVE_LIBZ3=1
  LDFLAGS_LIBZ3 += -Wl,-rpath,/usr/local/lib64
  CFLAGS_LIBZ3 += -I/usr/local/include
else ifneq ($(wildcard $(SYSROOT)/usr/include/z3++.h),)
  HAVE_LIBZ3=1
  LDFLAGS_LIBZ3 += -Wl,-rpath,/usr/lib
  CFLAGS_LIBZ3 += -I/usr/include
else
  HAVE_LIBZ3=0
endif

# test carl
ifneq ($(wildcard $(SYSROOT)/usr/local/include/carl/numbers/numbers.h),)
  HAVE_LIBCARL=1
  LDFLAGS_LIBCARL += -L/usr/local/lib
  CFLAGS_LIBCARL += -I/usr/local/include
else ifneq ($(wildcard $(SYSROOT)/usr/include/carl/numbers/numbers.h),)
  HAVE_LIBCARL=1
  LDFLAGS_LIBCARL += -L/usr/lib
  CFLAGS_LIBCARL += -I/usr/include
else
  HAVE_LIBCARL=0
endif

#check for gcc version to support c++14
GCCVERSION := $(shell expr `gcc -dumpversion | cut -f1 -d.` \>= 4.9)

ifeq ($(GCCVERSION),1)
	SUPPORT_CPP14=1
	CFLAGS_CPP14 = -std=c++14
else
	SUPPORT_CPP14=0
endif

