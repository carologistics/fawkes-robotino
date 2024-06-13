
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Worldmodel Test Plugin
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
#
#

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_robotino_wm_tester = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                          fawkesblackboard fawkesinterface \
                          RobotinoWorldModelInterface
OBJS_robotino_wm_tester = robotino_wm_tester_plugin.o \
                          robotino_wm_tester_thread.o

OBJS_all    = $(OBJS_robotino_wm-tester)
PLUGINS_all = $(PLUGINDIR)/robotino-wm-tester.$(SOEXT)
PLUGINS_build = $(PLUGINS_all)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_somewarning
warning_somewarning:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino Navigator plugin$(TNORMAL) (reason)"
endif

include $(BUILDSYSDIR)/base.mk
