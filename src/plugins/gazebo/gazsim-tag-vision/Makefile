#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-tag-vision Plugin
#
#   Created on Thu Nov 05 20:14:59 2015
#   Copyright (C) 2015 by Frederik Zwilling
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

LIBS_gazsim_tag_vision = fawkescore fawkesutils fawkesaspects fvutils \
                         fawkestf fawkesinterface fawkesblackboard llsf_msgs\
                         Position3DInterface TransformInterface \
                         SwitchInterface RobotinoLightInterface \
                         TagVisionInterface \
                         fawkesgazeboaspect
OBJS_gazsim_tag_vision = gazsim_tag_vision_plugin.o gazsim_tag_vision_thread.o

OBJS_all    = $(OBJS_gazsim_tag_vision)
PLUGINS_all = $(PLUGINDIR)/gazsim-tag-vision.$(SOEXT)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_TF),111)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) \
             -lm $(call boost-libs-ldflags,system) -lboost_system

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
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-tag-vision Plugin$(TNORMAL) " \
                         "(tf not available)"
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-tag-vision Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-tag-vision Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
