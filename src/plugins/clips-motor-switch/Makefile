#*****************************************************************************
#           Makefile Build System for Fawkes: CLIPS Agent Plugin
#                            -------------------
#   Created on Sat Jun 16 15:00:02 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/clips.mk

LIBS_clips_motor_switch = \
	fawkescore fawkesutils fawkesaspects fawkesclipsaspect \
	fawkesblackboard fawkesinterface MotorInterface
OBJS_clips_motor_switch = clips-motor-switch-plugin.o clips-motor-switch-thread.o

OBJS_all    = $(OBJS_clips_motor_switch)
PLUGINS_all = $(PLUGINDIR)/clips-motor-switch.$(SOEXT)

ifeq ($(HAVE_CLIPS),1)
  CFLAGS  += $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_CLIPS) \

  PLUGINS_build = $(PLUGINS_all)
else
  WARN_TARGETS = warning_clips
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-motor-switch plugin$(TNORMAL) ($(CLIPS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
