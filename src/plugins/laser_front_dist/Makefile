#*****************************************************************************
#         Makefile Build System for Fawkes: laser_front_dist Plugin
#                            -------------------
#   Created on Thu Jun 23 22:35:32 2016
 
#   Copyright (C) 2016 by Frederik Zwilling
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

LIBS_laser_front_dist = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface Laser360Interface \
											Position3DInterface

OBJS_laser_front_dist = laser_front_dist_plugin.o laser_front_dist_thread.o

PLUGINS_all = $(PLUGINDIR)/laser_front_dist.$(SOEXT)

OBJS_all = $(OBJS_laser_front_dist)

include $(BUILDSYSDIR)/base.mk
