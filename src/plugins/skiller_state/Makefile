
#*****************************************************************************
#         Makefile Build System for Fawkes: Skiller State LED
#                            -------------------
#   Created on Fri Jun 14 15:00:36 2019
#   Copyright (C) 2019 by Morian Sonnet
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

LIBS_skiller_state = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                 fawkesblackboard fawkesinterface \
                 SkillerInterface RobotinoSensorInterface
OBJS_skiller_state = skiller_state_plugin.o skiller_state_thread.o

CFLAGS += $(CFLAGS_CPP11)

OBJS_all    = $(OBJS_skiller_state)
PLUGINS_all = $(PLUGINDIR)/skiller_state.$(SOEXT)
PLUGINS_build = $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
