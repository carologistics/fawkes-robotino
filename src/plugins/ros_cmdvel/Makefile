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
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_ros_cmdvel = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect MotorInterface
OBJS_ros_cmdvel = ros_cmdvel_thread.o ros_cmdvel_plugin.o

OBJS_all = $(OBJS_ros_cmdvel)

ifeq ($(HAVE_ROS),1)
  #PRESUBDIRS += aspect
  CFLAGS  += $(CFLAGS_ROS)
  LDFLAGS += $(LDFLAGS_ROS)
  

  CFLAGS_ros_cmdvel_thread += $(call ros-pkg-cflags,geometry_msgs) $(CFLAGS) $(CFLAGS_TF)
  CFLAGS_ros_cmdvel_plugin = $(CFLAGS_odometry_thread) $(CFLAGS)
  LDFLAGS_ros_cmdvel += $(call ros-pkg-lflags,geometry_msgs) $(LDFLAGS_TF)
  

  
  
  PLUGINS_all = $(PLUGINDIR)/ros_cmdvel.so
  
 

else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Plugins$(TNORMAL) (ROS not found)"
endif

include $(BUILDSYSDIR)/base.mk
