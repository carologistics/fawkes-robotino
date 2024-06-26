#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-light-front Plugin
#
#   Created on Tue Aug 20 22:25:41 2013
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

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/fawkes/src/plugins/gazebo/gazebo.mk
include $(BUILDSYSDIR)/protobuf.mk
# include $(BASEDIR)/fawkes/src/plugins/gazebo/protobuf.mk

LIBS_gazsim_light_front = fawkescore fawkesutils fawkesaspects fvutils \
                          fawkestf fawkesinterface fawkesblackboard llsf_msgs \
                          Position3DInterface TransformInterface \
                          SwitchInterface RobotinoLightInterface \
                          fawkesgazeboaspect
OBJS_gazsim_light_front = gazsim_light_front_plugin.o \
                          gazsim_light_front_thread.o

OBJS_all    = $(OBJS_gazsim_light_front)
PLUGINS_all = $(PLUGINDIR)/gazsim-light-front.$(SOEXT)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_TF),111)
  CFLAGS  += $(CFLAGS_TF) -mno-avx
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  PLUGINS_build = $(PLUGINS_all)
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_gazebo warning_protobuf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-light-front Plugin$(TNORMAL) " \
                         "(tf not available)"
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-light-front Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-light-front Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
