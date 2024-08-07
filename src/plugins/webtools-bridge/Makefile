#*****************************************************************************
#               Makefile Build System for Fawkes: WebtoolsBridge Plugin
#
#   Created on Thu Sep 27 14:23:00 2012
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
##*****************************************************************************
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
include $(BUILDSYSDIR)/clips.mk
include $(BUILDSYSDIR)/boost.mk

LIBS_webtools_bridge = fawkescore fawkesutils fawkesaspects fvutils \
                       fawkestf fawkesinterface fawkesblackboard
OBJS_webtools_bridge = webtools_bridge_plugin.o webtools_bridge_thread.o \
                       event_emitter.o callable.o serializer.o \
                       websocket_server.o web_session.o proxy_session.o bridge_manager.o \
                       blackboard_processor.o rosbridge_proxy_processor.o clips_processor.o \
                       subscription_capability_manager.o subscription_capability.o \
                       advertisment_capability_manager.o advertisment_capability.o	\
                       service_capability_manager.o

OBJS_all    = $(OBJS_webtools_bridge)
PLUGINS_all = $(PLUGINDIR)/webtools-bridge.$(SOEXT)

ifneq ($(wildcard $(SYSROOT)/usr/include/websocketpp/version.hpp),)
	HAVE_WEBSOCKETPP:=1
else
	WEBSOCKETPP_ERROR:="Websocketpp_Not_Found"
endif

ifneq ($(wildcard $(SYSROOT)/usr/include/rapidjson/rapidjson.h),)
	HAVE_RAPIDJSON:=1
else
	RAPIDJSON_ERROR:="Rapidjson_Not_Found"
endif

CFLAGS_CPP11 += -std=c++11 -D_WEBSOCKETPP_CPP11_STL_

ifeq ($(HAVE_TF)$(HAVE_CLIPS)$(HAVE_CPP11)$(HAVE_WEBSOCKETPP)$(HAVE_RAPIDJSON),11111)
  CFLAGS  += $(CFLAGS_TF) $(CFLAGS_CLIPS) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_TF)

  PLUGINS_gui = $(PLUGINS_all)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS += warning_clips
  endif
  ifneq ($(HAVE_WEBSOCKETPP),1)
    WARN_TARGETS += warning_websocketpp
  endif
  ifneq ($(HAVE_RAPIDJSON),1)
    WARN_TARGETS += warning_rapidjson
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)
.PHONY: warning_tf warning_ros warning_geometry_msgs warning_cpp11 warning_clips warning_websocketpp
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting webtools-bridge libs" \
                         "$(TNORMAL) (tf not available)"
warning_cpp11:
	<$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webtools-bridge plugin$(TNORMAL) (C++11 not supported)"
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webtools-bridge plugin$(TNORMAL) ($(CLIPS_ERROR))"
warning_websocketpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webtools-bridge plugin$(TNORMAL) ($(WEBSOCKETPP_ERROR))"
warning_rapidjson:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webtools-bridge plugin$(TNORMAL) ($(RAPIDJSON_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
