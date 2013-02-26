#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Machinedetection Plugin
#                            -------------------
#   Created on Thu Feb 07 12:13:23 2013
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
include $(BUILDSYSDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS) $(CFLAGS_OPENCV)
LDFLAGS  += $(VISION_LDFLAGS) $(LDFLAGS_OPENCV)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_robotino_machinedetection = m \
	fawkescore fawkesutils fawkesaspects fawkesinterface fawkesblackboard \
	fawkesgeometry fvutils fvcams fvmodels fvclassifiers fvwidgets fvfilters\
	SwitchInterface Position3DInterface
OBJS_robotino_machinedetection = machinedetection_plugin.o machinedetection_thread.o Blob.o BlobFarm.o

OBJS_all    = $(OBJS_robotino_machinedetection)
PLUGINS_all = $(PLUGINDIR)/robotino-machinedetection.$(SOEXT)

include $(BUILDSYSDIR)/base.mk
