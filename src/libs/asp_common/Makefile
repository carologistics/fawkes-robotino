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
include $(BUILDSYSDIR)/protobuf.mk

LIBS_libasp_common = fawkes_protobuf_comm asp_msgs llsf_msgs fawkescore
OBJS_libasp_common = refbox_comm.o

OBJS_all = $(OBJS_libasp_common)

REQ_BOOST_LIBS = asio system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_CPP11)$(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),111)
  CFLAGS  += $(CFLAGS_PROTOBUF) $(CFLAGS_CPP11) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) \
		 $(CFLAGS_TF)
		 #-DNDEBUG
  LDFLAGS += $(LDFLAGS_PROTOBUF) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) \
		 $(LDFLAGS_CLINGO) $(LDFLAGS_TF)

  LIBS_all = $(LIBDIR)/libasp_common.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
	WARN_TARGETS_BOOST += $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_protobuf warning_clingo warning_cpp11 warning_tf
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP common library$(TNORMAL) (protobuf not found)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP common library$(TNORMAL) (C++11 not supported)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP common library$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk
