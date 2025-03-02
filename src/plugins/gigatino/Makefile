
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
               fawkesblackboard fawkesinterface ArduinoInterface \
               JoystickInterface
OBJS_arduino = arduino_plugin.o com_thread.o tf_thread.o com_message.o

CFLAGS += $(CFLAGS_TF)
LDFLAGS += $(LDFLAGS_TF)

OBJS_all      = $(OBJS_arduino)
PLUGINS_all   = $(PLUGINDIR)/arduino.$(SOEXT)

HAVE_ARDUINO = $(shell if [[ -a /dev/arduino ]]; then echo 1 ; else echo 0; fi)
HAVE_UDEV=$(if $(shell $(PKGCONFIG) --exists libudev; echo $${?/1/}),1,0)

ifeq ($(HAVE_CPP11)$(HAVE_TF)$(HAVE_UDEV),111)
  CFLAGS_UDEV=$(shell $(PKGCONFIG) --cflags libudev)
  LDFLAGS_UDEV=$(shell $(PKGCONFIG) --libs libudev)
  CFLAGS += $(CFLAGS_CPP11) $(CFLAGS_TF) $(CFLAGS_UDEV)
  LDFLAGS += $(LDFLAGS_CPP11) $(LDFLAGS_TF) $(LDFLAGS_UDEV)
  PLUGINS_build = $(PLUGINS_all)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_UDEV),1)
    WARN_TARGETS += warning_udev
  endif
endif

ifeq ($(HAVE_ARDUINO),1)
all: flash_arduino
else
  WARN_TARGETS += warning_arduino
all: $(WARN_TARGETS)
endif

flash_arduino:
	$(SILENT)($(MAKE) -C $(BASEDIR)/src/plugins/arduino/ArduinoSketch verify &> /dev/null) \
		&& echo -e "$(INDENT_PRINT)--> $(TGREEN)No need to reflash arduino $(TNORMAL)"\
		|| ( ($(MAKE) -C $(BASEDIR)/src/plugins/arduino/ArduinoSketch flash &> /dev/null) \
			&& echo -e "$(INDENT_PRINT)--> $(TGREEN)Successfully reflashed arduino $(TNORMAL)" \
			|| echo -e "$(INDENT_PRINT)--> $(TRED)Something went wrong while flashing the arduino $(TNORMAL)"; exit 1)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif

.PHONY: $(WARN_TARGETS)
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Arduino commnunication support$(TNORMAL) (C++11 support required)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Arduino commnunication support$(TNORMAL) (tf required)"
warning_arduino:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Not flashing arduino$(TNORMAL) (/dev/arduino not found)"
warning_udev:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Arduino commnunication support$(TNORMAL) (libudev not found)"
endif


include $(BUILDSYSDIR)/base.mk
