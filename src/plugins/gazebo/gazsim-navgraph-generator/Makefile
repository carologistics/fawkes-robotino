#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim Navgraph Generator Plugin
#
#   Created on Mon Feb 15 11:40:00 2016
#   Copyright (C) 2016 by David Schmidt
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

LIBS_gazsim_navgraph_generator = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard \
	       Position3DInterface fawkesgazeboaspect \
	       TransformInterface SwitchInterface \
	       RobotinoLightInterface llsf_msgs
OBJS_gazsim_navgraph_generator = gazsim_navgraph_generator_plugin.o gazsim_navgraph_generator_thread.o

OBJS_all    = $(OBJS_gazsim_navgraph_generator)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_TF),111)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-lbis-ldflags,system) -lboost_system

  PLUGINS_all = $(PLUGINDIR)/gazsim-navgraph-generator.$(SOEXT)
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_ros warning_geometry_msgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-navgraph-generator libs" \
                         "$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
