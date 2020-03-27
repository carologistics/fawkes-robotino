#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Fri 08 Mar 2020 17:33:33 CET
#   Copyright (C) 2020 by Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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
include $(BASEDIR)/etc/buildsys/gurobi.mk
include $(BUILDSYSDIR)/clips.mk

LIBS_clips_mip_scheduler = fawkescore fawkesclipsaspect
OBJS_clips_mip_scheduler = clips_mip_scheduler_plugin.o clips_mip_scheduler_thread.o \

CFLAGS += $(CFLAGS_CLIPS) $(CFLAGS_GUROBI)
LDFLAGS += $(LDFLAGS_CLIPS) $(LDFLAGS_GUROBI)

OBJS_all    = $(OBJS_clips_mip_scheduler)
PLUGINS_all = $(PLUGINDIR)/clips-mip-scheduler.$(SOEXT)

ifeq ($(HAVE_CLIPS)$(HAVE_GUROBI),11)
  PLUGINS_build = $(PLUGINS_all)
else
  ifneq ($(HAVE_GUROBI),1)
    WARN_TARGETS += warning_gurobi
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS += warning_clips
  endif
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips warning_gurobi
warning_gurobi:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-mip-scheduler plugin$(TNORMAL) ($(GUROBI_ERROR))"
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-mip-scheduler plugin$(TNORMAL) ($(CLIPS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
