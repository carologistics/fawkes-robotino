#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Ampelvar Plugin
#                            -------------------
#   Created on Thu May 24 17:09:43 2012
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
include $(BUILDSYSDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_robotino_ampelvar = m fawkescore fawkesutils fawkesaspects \
                         fawkesinterface fawkesblackboard \
                         fawkesgeometry fvutils fvcams fvmodels fvclassifiers \
                         SwitchInterface Position3DInterface
OBJS_robotino_ampelvar = robotino_ampelvar_plugin.o pipeline_thread.o

OBJS_all    = $(OBJS_robotino_ampelvar)
PLUGINS_all = $(PLUGINDIR)/robotino-ampelvar.$(SOEXT)
PLUGINS_build = $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
