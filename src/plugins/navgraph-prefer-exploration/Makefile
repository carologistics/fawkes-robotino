#*****************************************************************************
#     Makefile Build System for Fawkes: prefer exploration sub-graph
#                            -------------------
#   Created on Thu Apr 09 16:49:00 2015
#   copyright (C) 2006-2015 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/eigen3.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk

LIBS_navgraph_prefer_exploration = fawkescore fawkesutils fawkesaspects fawkesinterface \
		fawkesblackboard fawkestf fawkesnavgraph fawkesnavgraphaspect
OBJS_navgraph_prefer_exploration = navgraph_prefer_exploration_plugin.o \
				navgraph_prefer_exploration_thread.o
OBJS_all    = $(OBJS_navgraph_prefer_exploration)

ifeq ($(HAVE_NAVGRAPH),1)
  CFLAGS  += $(CFLAGS_NAVGRAPH)
  LDFLAGS += $(LDFLAGS_NAVGRAPH)

  PLUGINS_all = $(PLUGINDIR)/navgraph-prefer-exploration.$(SOEXT)
else
  # navgraph.mk warns already
endif

include $(BUILDSYSDIR)/base.mk
