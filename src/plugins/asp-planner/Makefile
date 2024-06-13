#*****************************************************************************
#           Makefile Build System for Fawkes: ASP Planner Plugin
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
include $(FAWKES_BASEDIR)/src/plugins/asp/clingo.mk
include $(FAWKES_BASEDIR)/src/plugins/robot-memory/robot_memory.mk

LIBS_asp_planner = fawkescore fawkesaspaspect fawkesaspects \
                   fawkesrobotmemory fawkesnavgraphaspect
OBJS_asp_planner = asp_planner_plugin.o asp_planner_thread.o \
                   asp_planner_clingo.o asp_planner_plan.o \
                   asp_planner_doc_and_config.o \
                   asp_planner_externals.o asp_planner_graph.o

OBJS_all    = $(OBJS_asp_planner)
PLUGINS_all = $(PLUGINDIR)/asp-planner.$(SOEXT)

REQ_BOOST_LIBS =
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_CPP14)$(HAVE_CLINGO)$(HAVE_BOOST_LIBS)$(HAVE_ROBOT_MEMORY),1111)
  CFLAGS  += $(CFLAGS_CPP14) $(CFLAGS_CLINGO) $(CFLAGS_ROBOT_MEMORY) \
             $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_CLINGO) $(LDFLAGS_ROBOT_MEMORY) \
             $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))
  PLUGINS_build = $(PLUGINS_all)
else
  ifneq ($(HAVE_CPP14),1)
		WARN_TARGETS += warning_cpp14
  endif
  ifneq ($(HAVE_CLINGO),1)
		WARN_TARGETS += warning_clingo
  endif
  ifneq ($(HAVE_ROBOT_MEMORY),1)
		WARN_TARGETS += warning_robot_memory
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
		WARN_TARGETS += $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)
warning_clingo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP planner plugin$(TNORMAL) ($(CLINGO_ERROR))"
warning_cpp14:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP planner plugin$(TNORMAL) (C++14 not supported)"
warning_robot_memory:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP planner plugin$(TNORMAL) (Robot Memory not available)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP planner plugin$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk
