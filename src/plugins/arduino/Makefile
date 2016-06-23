
#*****************************************************************************
#            Makefile Build System for Fawkes: Arduino Plugin
#                            -------------------
#   Created on Sun Nov 13 15:19:32 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces

# Uncomment to enable simple time tracking
#CFLAGS += -DUSE_TIMETRACKER

LIBS_arduino = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface ArduinoInterface
		
OBJS_arduino = arduino_plugin.o com_thread.o com_message.o

ifeq ($(HAVE_CPP11),1)

  CFLAGS += $(CFLAGS_CPP11)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
endif

OBJS_all = $(OBJS_arduino)
PLUGINS_all = $(PLUGINDIR)/arduino.$(SOEXT)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libudev warning_cpp11
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Arduino commnunication support$(TNORMAL) (C++11 support required)"
endif



include $(BUILDSYSDIR)/base.mk
