#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino OmniVision Plugin
#                            -------------------
#   Created on Thu May 24 17:09:43 2012
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
include $(BUILDSYSDIR)/fvconf.mk
include $(BUILDCONFDIR)/tf/tf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_omnivision_pucks = m \
	fawkescore fawkesutils fawkesaspects fawkesinterface fawkesblackboard \
	fawkesgeometry fvutils fvcams fvmodels fvclassifiers fvfilters \
	Position3DInterface SwitchInterface fawkestf 
OBJS_omnivision_pucks = omnivision_pucks_plugin.o pipeline_thread.o sensor_thread.o

OBJS_all    = $(OBJS_omnivision_pucks)

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,6),1)
    GCC_TOO_OLD=1
  endif
endif

ifneq ($(GCC_TOO_OLD),1)
  ifeq ($(HAVE_TF),11)
    CFLAGS  += $(CFLAGS_TF) $(CFLAGS_CPP11)
    LDFLAGS += $(LDFLAGS_TF)

    PLUGINS_all = $(PLUGINDIR)/omnivision-pucks.$(SOEXT)
  else
    WARN_TARGETS += warning_tf
  endif
else
  WARN_TARGETS += warning_old_gcc
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_old_gcc
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting omnivision-pucks plugin and libs$(TNORMAL) (tf not available)"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS Plugins$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.6)"

endif


include $(BUILDSYSDIR)/base.mk
