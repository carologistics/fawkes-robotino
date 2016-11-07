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

PLUGINS_all = $(PLUGINDIR)/mps-laser-gen.$(SOEXT)

OBJS_all = $(OBJS_mps-laser-gen)

include $(BUILDSYSDIR)/base.mk
