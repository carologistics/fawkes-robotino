
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

LIBS_machine_signal = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface fvutils fvcams fvmodels \
                      fvfilters fvclassifiers fawkesconfig fawkestf fvmodels \
                      RobotinoLightInterface SwitchInterface \
                      Position3DInterface SignalHintInterface LaserLineInterface
OBJS_machine_signal = custom_rois.o plugin.o state.o pipeline_thread.o sensor_thread.o

OBJS_all    = $(OBJS_machine_signal)
PLUGINS_all = $(PLUGINDIR)/machine-signal.$(SOEXT)

ifneq ($(GCC_TOO_OLD),1)
  PLUGINS_build = $(PLUGINS_all)
else
  WARN_TARGETS += warning_old_gcc
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_old_gcc
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS Plugins$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.7)"
endif

include $(BUILDSYSDIR)/base.mk
