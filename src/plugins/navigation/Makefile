
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Navigation Plugin
#                            -------------------
#   Created on Mon May 14 16:43:00 2012
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
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_robotino_navigator = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface MotorInterface \
		NavigatorInterface RobotinoSensorInterface Position3DInterface
OBJS_robotino_navigator = navigator_plugin.o navigator_thread.o

PLUGINS_all = $(PLUGINDIR)/robotino-navigator.$(SOEXT)

OBJS_all = $(OBJS_robotino_navigator)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_somewarning
warning_somewarning:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino Navigator plugin$(TNORMAL) (reason)"
endif

include $(BUILDSYSDIR)/base.mk
