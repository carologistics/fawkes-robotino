
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Template Plugin
#                            -------------------
#   Created on Wed May 23 18:31:45 2012
#   Copyright (C) 2012 by Daniel Ewert, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
# JUST REPLACE ALL OCCURENCES OF plugin-template AND REPLACE THEM ACCORDING TO YOUR NEEDS
#
#

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDCONFDIR)/tf/tf.mk

REQUIRED_PCL_LIBS = filters kdtree recognition

PRESUBDIRS = interfaces

LIBS_workpiece_pose = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                     fawkestf fawkesblackboard fawkesinterface  fawkesrosaspect \
                     SwitchInterface ConveyorPoseInterface LaserLineInterface \
		     WorkpiecePoseInterface

OBJS_workpiece_pose = workpiece_pose_plugin.o workpiece_pose_thread.o

OBJS_all    = $(OBJS_workpiece_pose)
PLUGINS_all = $(PLUGINDIR)/workpiece-pose.$(SOEXT)

CFLAGS  += $(CFLAGS_TF) $(CFLAGS_ROS) $(CFLAGS_workpiece_pose)
LDFLAGS += $(LDFLAGS_TF)

ifeq ($(HAVE_ROS),1)
    PLUGINS_build = $(PLUGINS_all)
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_ros
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting workpiece_pose Plugin$(TNORMAL) (ROS not found)"
endif

include $(BUILDSYSDIR)/base.mk
