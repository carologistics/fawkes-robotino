#*****************************************************************************
#     Makefile Build System for Fawkes: block paths based on laser clusters
#                            -------------------
#   Created on Sun Jul 13 15:19:29 2014
#   copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk

LIBS_navgraph_generator_2014 = fawkescore fawkesutils fawkesaspects fawkesinterface \
		fawkesblackboard fawkestf \
		fawkesnavgraph fawkesnavgraphaspect fawkesnavgraphgenerators \
		Position3DInterface
OBJS_navgraph_generator_2014 = navgraph_generator_llsf2014_plugin.o navgraph_generator_llsf2014_thread.o
OBJS_all    = $(OBJS_navgraph_generator_2014)

ifeq ($(HAVE_NAVGRAPH),1)
  CFLAGS  += $(CFLAGS_NAVGRAPH)
  LDFLAGS += $(LDFLAGS_NAVGRAPH)

  PLUGINS_all = $(PLUGINDIR)/navgraph-generator-2014.$(SOEXT)
else
  # navgraph.mk warns already
endif

include $(BUILDSYSDIR)/base.mk
