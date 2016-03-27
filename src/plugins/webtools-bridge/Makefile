  #*****************************************************************************
#               Makefile Build System for Fawkes: WebtooslBridge Plugin
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
OBJS_webtools_bridge = event_emitter.o callable.o webtools_bridge_plugin.o webtools_bridge_thread.o \
						Web_server.o  web_session.o bridge_manager.o subscription_capability_manager.o \
						subscription_capability.o blackboard_processor.o rosbridge_proxy_processor.o

OBJS_all    = $(OBJS_webtools_bridge)


CFLAGS_CPP11 += -std=c++11 -D_WEBSOCKETPP_CPP11_STL_

ifeq ($(HAVE_TF)$(HAVE_CLIPS)$(HAVE_CPP11),111)
  CFLAGS  += $(CFLAGS_TF) $(CFLAGS_CLIPS) $(CFLAGS_CPP11) 
  LDFLAGS += $(LDFLAGS_TF) 

  PLUGINS_all = $(PLUGINDIR)/webtools-bridge.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_ros warning_geometry_msgs warning_cpp11 warning_clips
warning_tf: 
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting webtools-bridge libs" \
                         "$(TNORMAL) (tf not available)"
warning_cpp11:
	<$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webtools-bridge plugin$(TNORMAL) (C++11 not supported)"
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webtools-bridge plugin$(TNORMAL) ($(CLIPS_ERROR))"
endif


##===>Stupid temp..find another way to do it
#$(TARGET).so: .objs_fawkes/$(TARGET).o

include $(BUILDSYSDIR)/base.mk


#$(TARGET).so: .objs_fawkes/$(TARGET).o
	# g++ -shared -Wl,--export-dynamic .objs_fawkes/$(TARGET).o $(LDFLAGS) -o $(TARGET).so


