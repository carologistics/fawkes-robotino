
#*****************************************************************************
#            Makefile Build System for Fawkes: gazsim-gripper Plugin
#                            -------------------
#   Created on Mon Apr 20 18:16:12 2015
#   Copyright (C) 2015 Frederik Zwilling
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/fawkes/src/plugins/gazebo/gazebo.mk
include $(BUILDSYSDIR)/protobuf.mk


LIBS_gazsim_gripper = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		fawkesinterface fawkesconfig AX12GripperInterface LedInterface \
	       	JointInterface DynamixelServoInterface gazsim_msgs \
		fawkesgazeboaspect
OBJS_gazsim_gripper = gazsim_gripper_plugin.o gazsim_gripper_thread.o

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF),11)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  PLUGINS_all = $(PLUGINDIR)/gazsim-light-front.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_gazebo warning_protobuf
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-gripper Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-gripper Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
endif


PLUGINS_all = $(PLUGINDIR)/gazsim-gripper.so
OBJS_all = $(OBJS_gazsim_gripper)

include $(BUILDSYSDIR)/base.mk

