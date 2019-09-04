
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

HAVE_arduino = $(shell [ -a /dev/arduino ]; echo $$?)

PRESUBDIRS = interfaces

# Uncomment to enable simple time tracking
#CFLAGS += -DUSE_TIMETRACKER

LIBS_arduino = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
               fawkesblackboard fawkesinterface ArduinoInterface \
               JoystickInterface
OBJS_arduino = arduino_plugin.o com_thread.o tf_thread.o com_message.o

CFLAGS += $(CFLAGS_TF)
LDFLAGS += $(LDFLAGS_TF)

OBJS_all      = $(OBJS_arduino)
PLUGINS_all   = $(PLUGINDIR)/arduino.$(SOEXT)
PLUGINS_build = $(PLUGINS_all)

ifeq ($(HAVE_CPP11),1)
  CFLAGS += $(CFLAGS_CPP11)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
endif

ifeq ($(HAVE_TF),1)
  CFLAGS += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
else
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifneq ($(HAVE_arduino),0) # arduino not found
	WARN_TARGETS += warning_arduino
else 
  ifeq ($(HAVE_arduino),0) #arduino was found
all: flash_arduino
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_tf warning_cpp11 warning_arduino flash_arduino
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Arduino commnunication support$(TNORMAL) (C++11 support required)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Arduino commnunication support$(TNORMAL) (tf required)"
warning_arduino:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting flashing Arduino$(TNORMAL) (/dev/arduino not found)"
flash_arduino:
	$(SILENT)($(MAKE) -C $(BASEDIR)/src/plugins/arduino/ArduinoSketch verify &> /dev/null) \
		&& echo -e "$(INDENT_PRINT)--> $(TGREEN)No need to reflash arduino $(TNORMAL)"\
		|| ( ($(MAKE) -C $(BASEDIR)/src/plugins/arduino/ArduinoSketch flash &> /dev/null) \
			&& echo -e "$(INDENT_PRINT)--> $(TGREEN)Successfully reflashed arduino $(TNORMAL)" \
			|| echo -e "$(INDENT_PRINT)--> $(TRED)Something went wrong while flashing the arduino $(TNORMAL)")
endif



include $(BUILDSYSDIR)/base.mk
