
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
include $(BUILDSYSDIR)/ros2.mk
include $(BUILDCONFDIR)/tf/tf.mk

HAVE_BULLET_VERSION = $(if $(shell $(PKGCONFIG) --atleast-version=2.82 'bullet'; echo $${?/1/}),1,0)

REQUIRED_PCL_LIBS = sample_consensus segmentation filters surface features

#The necessary libraries your plugin needs.
LIBS_conveyor_plane = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                     fawkestf fawkesblackboard fawkesinterface fawkespcl_utils fawkesros2aspect \
                     SwitchInterface Position3DInterface LaserLineInterface
#binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
OBJS_conveyor_plane = conveyor_plane_plugin.o conveyor_plane_thread.o

OBJS_all    = $(OBJS_conveyor_plane)
PLUGINS_all = $(PLUGINDIR)/conveyor-plane.$(SOEXT)

CFLAGS  += $(CFLAGS_TF) $(CFLAGS_PCL) $(CFLAGS_ROS2) \
           $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS)) $(VISUALIZATION_MSGS_cflags) $(PCL_CONVERSIONS_cflags) $(GEOMETRY_MSGS_cflags)
LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) $(LDFLAGS_ROS2) \
           $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS)) $(VISUALIZATION_MSGS_lflags) $(PCL_CONVERSIONS_lflags) $(GEOMETRY_MSGS_lflags)

PRESUBDIRS = interfaces

ifeq ($(HAVE_ROS2)$(HAVE_TF)$(HAVE_PCL)$(call pcl-have-libs,$(REQUIRED_PCL_LIBS)),1111)
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
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting conveyor_plane Plugin$(TNORMAL) (bullet version < 2.82)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting conveyor_plane Plugin$(TNORMAL) (ROS not found)"
endif

include $(BUILDSYSDIR)/base.mk
