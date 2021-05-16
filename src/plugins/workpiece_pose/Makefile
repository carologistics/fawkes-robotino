
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDCONFDIR)/tf/tf.mk

HAVE_BULLET_VERSION = $(if $(shell $(PKGCONFIG) --atleast-version=2.82 'bullet'; echo $${?/1/}),1,0)

REQUIRED_PCL_LIBS = filters kdtree recognition

PRESUBDIRS = interfaces

LIBS_workpiece_pose = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                     fawkestf fawkesblackboard fawkesinterface fawkespcl_utils fawkesrosaspect \
                     SwitchInterface ConveyorPoseInterface LaserLineInterface
OBJS_workpiece_pose = workpiece_pose_plugin.o workpiece_pose_thread.o

OBJS_all    = $(OBJS_workpiece_pose)
PLUGINS_all = $(PLUGINDIR)/workpiece-pose.$(SOEXT)

CFLAGS  += $(CFLAGS_TF) $(CFLAGS_PCL) $(CFLAGS_ROS) $(CFLAGS_workpiece_pose) $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS))
LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))

ifeq ($(HAVE_ROS),1)
  ifeq ($(HAVE_BULLET_VERSION),1)
    PLUGINS_build = $(PLUGINS_all)
  else
    WARN_TARGETS += warning_bullet
  endif
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_bullet warning_ros
warning_bullet:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting workpiece_pose Plugin$(TNORMAL) (bullet version < 2.82)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting workpiece_pose Plugin$(TNORMAL) (ROS not found)"
endif

include $(BUILDSYSDIR)/base.mk
