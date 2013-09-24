#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-llsf-statistics Plugin
#
#   Created on Mon Sep 23 17:11:35 2013
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#                         Frederik Zwilling
#
##*****************************************************************************
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
include $(BASEDIR)/fawkes/src/plugins/gazebo/gazebo.mk
include $(BASEDIR)/fawkes/src/plugins/gazebo/protobuf.mk

LIBS_gazsim_llsf_statistics = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard protobufmsgs\
	       Position3DInterface TransformInterface \
	       SwitchInterface RobotinoLightInterface \
	       fawkesgazeboaspect
OBJS_gazsim_llsf_statistics = gazsim_llsf_statistics_plugin.o gazsim_llsf_statistics_thread.o

OBJS_all    = $(OBJS_gazsim_llsf_statistics)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  PLUGINS_all = $(PLUGINDIR)/gazsim-llsf-statistics.so
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting llsf-statistics-Sim libs" \
                         "$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
