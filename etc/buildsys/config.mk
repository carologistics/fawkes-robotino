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

BUILDSYSDIR            = $(abspath $(TOP_BASEDIR)/fawkes/etc/buildsys)
SECONDARY_BUILDSYSDIR  = $(abspath $(TOP_BASEDIR)/etc/buildsys)

ifeq ($(wildcard $(BUILDSYSDIR)/config.mk),)
  $(error Fawkes submodule missing. Execute "git submodule update --init")
else
  include $(BUILDSYSDIR)/config.mk
endif

endif # __robotino_config_mk_

