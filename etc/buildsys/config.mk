#*****************************************************************************
#                      Makefile Build System for Fawkes
#                               -------------------
#   Created on Mon May 14 11:49:00 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __robotino_config_mk_
__robotino_config_mk_ := 1

TOP_BASEDIR ?= $(BASEDIR)

# Due to a limitation in GNU Make,
# cf. http://savannah.gnu.org/bugs/?712
ifneq ($(words $(abspath $(TOP_BASEDIR))),1)
  $(error Path to Fawkes may not contain spaces. \
          Move Fawkes to another location and call make again)
endif

BUILDSYSDIR            = $(abspath $(TOP_BASEDIR)/fawkes/etc/buildsys)
SECONDARY_BUILDSYSDIR  = $(abspath $(TOP_BASEDIR)/etc/buildsys)

# Uncomment to use clang as the compiler
# After enabled you MUST clean and rebuild your whole tree
#CC=clang
#LD=clang

ifeq ($(wildcard $(BUILDSYSDIR)/config.mk),)
  $(error Fawkes submodule missing. Execute "git submodule update --init")
else
  ifndef __buildsys_config_mk_
    include $(BUILDSYSDIR)/config.mk
  endif
endif

# Globally enable optimization for the Robotino platform
CFLAGS_EXTRA  += -g -Wall -Werror -O2

endif # __robotino_config_mk_

