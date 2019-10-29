
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
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_gazsim_gripper = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
                      fawkesinterface fawkesconfig gazsim_msgs \
                      fawkesgazeboaspect ArduinoInterface
OBJS_gazsim_gripper = gazsim_gripper_plugin.o gazsim_gripper_thread.o

OBJS_all    = $(OBJS_gazsim_gripper)
PLUGINS_all = $(PLUGINDIR)/gazsim-gripper.$(SOEXT)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF),11)
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
endif

ifeq ($(HAVE_TF),1)
  CFLAGS += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
else
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
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
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting gazsim-gripper Plugin$(TNORMAL) (tf required)"

endif

include $(BUILDSYSDIR)/base.mk

