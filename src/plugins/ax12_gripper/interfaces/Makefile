#*****************************************************************************
#              Makefile Build System for Fawkes: PTU interfaces
#                            -------------------
#   Created on Sat Feb 28 10:41:48 2015
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
#                      2015    Nicolas Limpert
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk

INTERFACES_all = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))
include $(BUILDSYSDIR)/interface.mk

include $(BUILDSYSDIR)/base.mk
