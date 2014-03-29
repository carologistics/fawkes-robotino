#*****************************************************************************
#           Makefile Build System for Fawkes: NavGraph Broker
#                            -------------------
#  Created on Sat March 29th 12:24:01 2014
#  Copyright (C) 2014 by Sebastian reuter, Carologistics RoboCup Team]
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

CFLAGS += $(CFLAGS_CPP11)

LIBS_navgraph_broker = fawkescore fawkesutils fawkesaspects fawkesnavgraphaspect fawkesconstraintrepo
OBJS_navgraph_broker = navgraph_broker_plugin.o navgraph_broker_thread.o

OBJS_all = $(OBJS_navgraph_broker)

PLUGINS_all = $(PLUGINDIR)/navgraph-broker.$(SOEXT)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
#.PHONY: warning_ros warning_clips warning_xmlrpcpp $(WARN_TARGETS_BOOST)
endif

include $(BUILDSYSDIR)/base.mk
