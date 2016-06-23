
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
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces

REQUIRED_PCL_LIBS = sample_consensus segmentation filters surface features

#The necessary libraries your plugin needs.
LIBS_conveyor_pose = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkestf fawkesblackboard fawkesinterface fawkespcl_utils fawkesrosaspect \
		SwitchInterface ConveyorConfigInterface Position3DInterface LaserLineInterface
#binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
OBJS_conveyor_pose = conveyor_pose_plugin.o conveyor_pose_thread.o

OBJS_all = $(OBJS_conveyor_pose)

CFLAGS  += $(CFLAGS_TF) $(CFLAGS_PCL) $(CFFLAGS_conveyor_pose) $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS))
LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))

#names of the plugins you want to build
PLUGINS_all = $(PLUGINDIR)/conveyor-pose.$(SOEXT)

include $(BUILDSYSDIR)/base.mk
