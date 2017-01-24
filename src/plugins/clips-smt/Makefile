#*****************************************************************************
#           Makefile Build System for Fawkes: CLIPS Smt Plugin
#                            -------------------
#   Created on Fry Dec 16 14:38 2016 by Igor Nicolai Bongartz
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
include $(BUILDSYSDIR)/clips.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk

CFLAGS_LIBZ3  += -I/usr/local/include
LDFLAGS_LIBZ3 += -Wl,-rpath,/usr/local/lib64

CFLAGS += $(CFLAGS_LIBZ3)
LDFLAGS += $(LDFLAGS_LIBZ3)

LIBS_clips_smt = fawkescore fawkesutils fawkesaspects \
		      fawkesclipsaspect fawkesnavgraphaspect \
		      fawkesnavgraph \
			  z3
OBJS_clips_smt = clips_smt_plugin.o clips_smt_thread.o

OBJS_all = $(OBJS_clips_navgraph)

ifeq ($(HAVE_CPP11)$(HAVE_CLIPS)$(HAVE_NAVGRAPH),111)
  CFLAGS  += $(CFLAGS_CLIPS) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/clips-smt.$(SOEXT)

else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS = warning_navgraph
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips warning_cpp11
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-smt plugin$(TNORMAL) ($(CLIPS_ERROR))"

warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-smt plugin$(TNORMAL) (C++11 not supported)"

warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-smt plugin$(TNORMAL) (Navgraph not available)"

endif

include $(BUILDSYSDIR)/base.mk
