
#*****************************************************************************
#            Makefile Build System for Fawkes: gazsim-conveyor Plugin
#                            -------------------
#   Created on Fri Jul 10 11:27:12 2015
#   Copyright (C) 2015 Randolph Maaßen
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


LIBS_gazsim_conveyor = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		fawkesinterface fawkesconfig gazsim_msgs \
		fawkesgazeboaspect \
		llsf_msgs
OBJS_gazsim_conveyor = gazsim_conveyor_plugin.o gazsim_conveyor_thread.o

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF),11)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  PLUGINS_all = $(PLUGINDIR)/gazsim-conveyor.so
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
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-conveyor Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-conveyor Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
endif

OBJS_all = $(OBJS_gazsim_conveyor)

include $(BUILDSYSDIR)/base.mk

