#*****************************************************************************
#           Makefile Build System for Fawkes: ASP Planer Plugin
#                            -------------------
#   Created on Thu Aug 18 04:20:02 2016
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

LIBS_asp_planer = fawkescore fawkesaspaspect fawkesaspects fawkesrobotmemory fawkesnavgraphaspect
OBJS_asp_planer = asp_planer_plugin.o asp_planer_thread.o asp_planer_clingo.o asp_planer_plan.o\
				  asp_planer_doc_and_config.o asp_planer_externals.o asp_planer_graph.o

OBJS_all = $(OBJS_asp_planer)

REQ_BOOST_LIBS = 
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

#TODO: export this in .mk and really check
HAVE_CLINGO = 1
CFLAGS_CLINGO = -DWITH_THREADS=1 \
	-I $(HOME)/clingo/libclasp \
	-I $(HOME)/clingo/libclingo \
	-I $(HOME)/clingo/libgringo \
	-I $(HOME)/clingo/liblp \
	-I $(HOME)/clingo/libprogram_opts
LDFLAGS_CLINGO = -L$(HOME)/clingo-build/bin/ -Wl,-rpath=$(HOME)/clingo-build/bin/ -lclingo

#Actually we use C++14, but there is no fawkes flag for that.
ifeq ($(HAVE_CPP11)$(HAVE_CLINGO)$(HAVE_BOOST_LIBS),111)
  CFLAGS  += $(CFLAGS_CPP11) \
		 $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) \
		 $(CFLAGS_CLINGO) -Wno-error=comment
		 #-DNDEBUG
  LDFLAGS += $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) \
		 $(LDFLAGS_CLINGO)

  PLUGINS_all = $(PLUGINDIR)/asp-planer.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_CLINGO),1)
	WARN_TARGETS += warning_clingo
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
	WARN_TARGETS_BOOST += $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_clingo warning_cpp11
warning_clingo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP planer plugin$(TNORMAL) ($(CLINGO_ERROR))"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP planer plugin$(TNORMAL) (C++11 not supported)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP planer plugin$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk
