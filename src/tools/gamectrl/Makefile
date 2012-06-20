
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino GameCtrl Tool
#                            -------------------
#   Created on Tue Jun 19 20:36:45 2012
#   Copyright (C) 2012 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

LIBS_gamectrl = m fawkescore fawkesutils  \
		fawkesblackboard fawkesinterface SwitchInterface
OBJS_gamectrl = gamectrl.o

BINS_all = $(BINDIR)/gamectrl
OBJS_all = $(OBJS_gamectrl)

include $(BUILDSYSDIR)/base.mk
