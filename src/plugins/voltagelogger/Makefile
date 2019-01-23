
#*****************************************************************************
#         Makefile Build System for Fawkes: Voltage logger Plugin
#                            -------------------
#   Created on Wed May 23 18:31:45 2012
#   Copyright (C) 2012 by Daniel Ewert, Carologistics RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_voltagelogger = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                     fawkesblackboard fawkesinterface fawkeslogging \
                     BatteryInterface
OBJS_voltagelogger = voltagelogger_plugin.o voltagelogger_thread.o

OBJS_all    = $(OBJS_voltagelogger)
PLUGINS_all = $(PLUGINDIR)/voltagelogger.$(SOEXT)
PLUGINS_build = $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
