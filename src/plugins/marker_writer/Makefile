
#*****************************************************************************
#         Makefile Build System for Fawkes: Marker Writer Plugin
#                            -------------------
#   Created on Wed April 24 19:55:45 2013
#   Copyright (C) 2013 by Daniel Ewert, Carologistics RoboCup Team
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
include $(BUILDSYSDIR)/ros.mk

LIBS_marker_writer = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                     fawkesblackboard fawkesinterface fawkesrosaspect \
                     Position3DInterface
OBJS_marker_writer = marker_writer_plugin.o marker_writer_thread.o

OBJS_all    = $(OBJS_marker_writer)
PLUGINS_all = $(PLUGINDIR)/marker-writer.$(SOEXT)

ifeq ($(HAVE_ROS)$(HAVE_CPP11),11)
  ifeq ($(call ros-have-pkg,visualization_msgs),1)
    CFLAGS += $(CFLAGS_CPP11) \
              $(CFLAGS_ROS) $(call ros-pkg-cflags,visualization_msgs)
    LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs)

    PLUGINS_build = $(PLUGINS_all)
  else 
    WARN_TARGETS+= warning_visualization_msgs
  endif
else
  WARN_TARGETS+= warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros warning_visualization_msgs
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)ROS not available$(TNORMAL)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)ROS package visualization_msgs not available$(TNORMAL) "

endif

include $(BUILDSYSDIR)/base.mk
