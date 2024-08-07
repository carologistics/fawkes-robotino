
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Front Light Plugin
#                            -------------------
#   Created on Wed May 23 18:31:45 2012
#   Copyright (C) 2013 Tobias Neumann, Carologistics RoboCup Team
#                 2013 Florian Nolden, Carologistics RoboCup Team
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
include $(BUILDSYSDIR)/fvconf.mk
include $(BUILDSYSDIR)/boost.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_light_front = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                   fawkesblackboard fawkesinterface fvutils fvcams fvmodels \
                   fvclassifiers fvfilters fawkestf TransformInterface \
                   RobotinoLightInterface Position3DInterface SwitchInterface
OBJS_light_front = light_front.o light_front_thread.o

OBJS_all    = $(OBJS_light_front)
PLUGINS_all = $(PLUGINDIR)/light_front.$(SOEXT)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)

  PLUGINS_build = $(PLUGINS_all)
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_somewarning
warning_somewarning:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino Navigator plugin$(TNORMAL) (reason)"
endif

include $(BUILDSYSDIR)/base.mk
