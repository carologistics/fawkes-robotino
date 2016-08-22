#*****************************************************************************
#           Makefile Build System for Fawkes: ASP Agent Plugin
#                            -------------------
#   Created on Mon Aug 15 16:33:02 2016
#   Copyright (C) 2016 by Björn Schäpers
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
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/protobuf.mk

LIBS_asp_agent = asp_common fawkes_protobuf_comm llsf_msgs fawkescore asp_msgs
OBJS_asp_agent = asp_agent_plugin.o asp_agent_thread.o refbox_comm.o

OBJS_all = $(OBJS_asp_agent)

REQ_BOOST_LIBS =
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_CPP11)$(HAVE_BOOST_LIBS)$(HAVE_PROTOBUF)$(HAVE_TF),1111)
  CFLAGS  += $(CFLAGS_PROTOBUF) $(CFLAGS_CPP11) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) \
		 $(CFLAGS_TF)
		 #-DNDEBUG
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) \
		 $(LDFLAGS_CLINGO) $(LDFLAGS_TF)

  PLUGINS_all = $(PLUGINDIR)/asp-agent.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_PROTOBUF),1)
	WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_TF),1)
	WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
	WARN_TARGETS_BOOST += $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_protobuf warning_clingo warning_cpp11 warning_tf
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP agent plugin$(TNORMAL) (C++11 not supported)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP common library$(TNORMAL) (protobuf not found)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP agent plugin$(TNORMAL) (fawkestf not available)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP agent plugin$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk
