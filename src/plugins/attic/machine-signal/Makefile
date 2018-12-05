
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Template Plugin
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
# JUST REPLACE ALL OCCURENCES OF plugin-template AND REPLACE THEM ACCORDING TO YOUR NEEDS
#
#

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/fvconf.mk
include $(BUILDCONFDIR)/tf/tf.mk

CFLAGS   += $(VISION_CFLAGS) $(CFLAGS_CPP11) $(CFLAGS_TF)
LDFLAGS  += $(VISION_LDFLAGS) $(LDFLAGS_TF)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,7),1)
    GCC_TOO_OLD=1
  endif
endif

PRESUBDIRS = interfaces

#The necessary libraries your plugin needs.
LIBS_machine_signal = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface fvutils fvcams fvmodels \
		fvfilters fvclassifiers fawkesconfig RobotinoLightInterface SwitchInterface \
		Position3DInterface SignalHintInterface LaserLineInterface fawkestf fvmodels
#binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
OBJS_machine_signal = custom_rois.o plugin.o state.o pipeline_thread.o sensor_thread.o

ifneq ($(GCC_TOO_OLD),1)
  PLUGINS_all = $(PLUGINDIR)/machine-signal.$(SOEXT)
else
  WARN_TARGETS += warning_old_gcc
endif

OBJS_all = $(OBJS_machine_signal)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_old_gcc
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS Plugins$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.7)"
endif

include $(BUILDSYSDIR)/base.mk
