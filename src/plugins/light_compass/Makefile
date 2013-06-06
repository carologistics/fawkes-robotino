
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Light Plugin
#                            -------------------
#   Created on Wed Feb 27 14:21:44 2013
#   Copyright (C) 2013 by Florian Nolden & Tobias Neumann, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
# 
#
#

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

#The necessary libraries your plugin needs.
LIBS_light_compass = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface fvutils fvcams fvmodels fvclassifiers fawkesgeometry Position3DInterface fvfilters
#binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
OBJS_light_compass = light_compass.o light_compass_thread.o ColorModelRange.o

#names of the plugins you want to build
PLUGINS_all = $(PLUGINDIR)/light_compass.$(SOEXT)

OBJS_all = $(OBJS_light_compass)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_somewarning
warning_somewarning:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino Navigator plugin$(TNORMAL) (reason)"
endif

include $(BUILDSYSDIR)/base.mk
