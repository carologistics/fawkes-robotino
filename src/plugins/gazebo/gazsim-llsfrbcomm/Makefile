#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-llsfrbcomm 
#                            -------------------
#   Created on Sat Jun 16 14:42:34 2012
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

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk
include $(BASEDIR)/fawkes/src/plugins/gazebo/gazebo.mk
include $(BUILDSYSDIR)/protobuf.mk
# include $(BASEDIR)/fawkes/src/plugins/gazebo/protobuf.mk


CFLAGS += $(CFLAGS_CPP11)

LIBS_gazsim_llsfrbcomm = fawkescore fawkesutils fawkesaspects llsf_msgs fawkesgazeboaspect gazsim_msgs
OBJS_gazsim_llsfrbcomm = gazsim_llsfrbcomm_plugin.o gazsim_llsfrbcomm_thread.o

OBJS_all    = $(OBJS_gazsim_llsfrbcomm)
PLUGINS_all = $(PLUGINDIR)/gazsim-llsfrbcomm.$(SOEXT)

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_PROTOBUF_COMM)$(HAVE_BOOST_LIBS),1111)
  CFLAGS  += $(CFLAGS_PROTOBUF) $(CFLAGS_PROTOBUF_COMM) \
             $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) \
             $(CFLAGS_GAZEBO)
  LDFLAGS += $(LDFLAGS_PROTOBUF) $(LDFLAGS_PROTOBUF_COMM) \
             $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) \
             $(LDFLAGS_GAZEBO)

  PLUGINS_build = $(PLUGINS_all)
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_PROTOBUF_COMM),1)
    WARN_TARGETS += warning_protobuf_comm
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS += warning_boost
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-webcam Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-webcam Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
warning_protobuf_comm:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-webcam Plugin$(TNORMAL) " \
		"(protobuf_comm[-devel] not installed)"
warning_boost:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-webcam Plugin$(TNORMAL) " \
		"(req. boost libs not found)"
endif


include $(BUILDSYSDIR)/base.mk

