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
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/ros.mk

CFLAGS += $(CFLAGS_CPP11)

LIBS_navgraph_broker = fawkescore fawkesutils fawkesaspects fawkesinterface fawkesnavgraphaspect \
						fawkesnavgraphconstraints fawkesblackboard NavPathInterface  \
						fawkes_protobuf_comm fawkesgossip fawkesgossipaspect navgrap_broker_msgs
OBJS_navgraph_broker = navgraph_broker_plugin.o navgraph_broker_thread.o

OBJS_all = $(OBJS_navgraph_broker)

REQ_BOOST_LIBS = system signals2
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_CPP11)$(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),111)
  CFLAGS  += $(CFLAGS_PROTOBUF) $(CFLAGS_CPP11) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

  PLUGINS_all = $(PLUGINDIR)/navgraph-broker.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif


# Protobuf messages
PROTOBUF_all = navgrap_broker_msgs
MSGS_navgrap_broker_msgs = $(notdir $(patsubst %.proto,%,$(wildcard $(SRCDIR)/*.proto)))

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
#.PHONY: warning_ros warning_clips warning_xmlrpcpp $(WARN_TARGETS_BOOST)
endif

include $(BUILDSYSDIR)/protobuf_msgs.mk
include $(BUILDSYSDIR)/base.mk
