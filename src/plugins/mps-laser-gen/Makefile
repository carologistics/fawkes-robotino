#*****************************************************************************
#         Makefile Build System for Fawkes: mps-laser-gen Plugin
#                            -------------------
#   Created on Thu Jun 30 21:54:46 2016
 
#   Copyright (C) 2016 by Tim Niemueller
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
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/eigen3.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_mps_laser_gen = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface fawkesrosaspect

OBJS_mps_laser_gen = mps-laser-gen_plugin.o mps-laser-gen_thread.o

CFLAGS +=  $(CFLAGS_TF) $(CFLAGS_NAVGRAPH) $(CFLAGS_EIGEN3) $(CFLAGS_ROS) $(call ros-pkg-cflags,visualization_msgs)
LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_NAVGRAPH) $(LDFLAGS_EIGEN3) $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs)

ifeq ($(HAVE_ROS)$(call ros-have-pkg,visualization_msgs),11)
  PLUGINS_all = $(PLUGINDIR)/mps-laser-gen.$(SOEXT)
else
  ifneq ($(HAVE_ROS),1)
    WARN_TARGETS += warning_ros
  endif
  ifneq ($(call ros-have-pkg,visualization_msgs),1)
    WARN_TARGETS += warning_visualization_msgs
  endif
endif

OBJS_all = $(OBJS_mps-laser-gen)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros warning_visualization_msgs
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting mps-laser-gen plugin$(TNORMAL) (ROS not found)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting mps-laser-gen plugin$(TNORMAL) (ROS package visualization_msgs not found)"
endif

include $(BUILDSYSDIR)/base.mk
