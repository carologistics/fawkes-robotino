#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-machine-signal Plugin
#
#   Created on Thu Apr 02 14:39:52 2015
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
# include $(BASEDIR)/fawkes/src/plugins/gazebo/protobuf.mk

LIBS_gazsim_machine_signal = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard llsf_msgs\
	       Position3DInterface TransformInterface \
	       SwitchInterface RobotinoLightInterface \
	       fawkesgazeboaspect gazsim_msgs
OBJS_gazsim_machine_signal = gazsim_machine_signal_plugin.o gazsim_machine_signal_thread.o

OBJS_all    = $(OBJS_gazsim_machine_signal)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_TF),111)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  PLUGINS_all = $(PLUGINDIR)/gazsim-machine-signal.so
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
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-machine-signal Plugin$(TNORMAL) " \
                         "(tf not available)"
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-machine-signal Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-machine-signal Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
