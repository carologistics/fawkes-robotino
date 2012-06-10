#*****************************************************************************
#               Makefile Build System for Fawkes: ROS Plugin
#                            -------------------
#   Created on Thu May 05 18:23:31 2011
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_ros_navigator = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect NavigatorInterface
OBJS_ros_navigator = ros_navigator_plugin.o ros_navigator_thread.o

OBJS_all = $(OBJS_ros_navigator)

HAVE_BOOST_THREAD = $(call boost-have-lib,thread)

ifeq ($(HAVE_ROS),1)
  #PRESUBDIRS += aspect
  CFLAGS  += $(CFLAGS_ROS) $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_ROS) $(LDFLAGS_TF)

  ifeq ($(HAVE_BOOST_THREAD),1)
    CFLAGS_ros_navigator_thread += $(CFLAGS) \
	$(call ros-pkg-cflags,move_base_msgs) \
        $(call ros-pkg-lflags,actionlib) $(call boost-lib-cflags,thread)
    CFLAGS_ros_navigator_plugin = $(CFLAGS_ros_navigator_thread)
    LDFLAGS_ros_navigator += $(call ros-pkg-lflags,move_base_msgs) \
      $(call ros-pkg-lflags,actionlib) $(call boost-lib-ldflags,thread) -lpthread

    PLUGINS_all = $(PLUGINDIR)/ros_navigator.so
  else
   WARN_TARGETS += warning_boost
  endif
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros warning_boost
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Plugins$(TNORMAL) (ROS not found)"
warning_boost:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Plugins$(TNORMAL) (Boost Thread library not found)"
endif

include $(BUILDSYSDIR)/base.mk
