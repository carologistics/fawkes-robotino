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
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/clips.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk

#CFLAGS += $(CFLAGS_LIBZ3) $(CFLAGS_LIBCARL)
#LDFLAGS += $(LDFLAGS_LIBZ3) $(LDFLAGS_LIBCARL)

LIBS_clips_smt = fawkescore fawkesutils fawkesaspects \
		      fawkesclipsaspect fawkesnavgraphaspect \
		      fawkesnavgraph \
			  z3 \
			  fawkestf
OBJS_clips_smt = clips_smt_plugin.o clips_smt_thread.o

OBJS_all = $(OBJS_clips_smt)

ifeq ($(SUPPORT_CPP14)$(HAVE_CLIPS)$(HAVE_NAVGRAPH)$(HAVE_LIBZ3)$(HAVE_LIBCARL)$(HAVE_TF),111111)
  CFLAGS  += $(CFLAGS_CLIPS) $(CFLAGS_LIBZ3) $(CFLAGS_LIBCARL) $(CFLAGS_CPP14) $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_CLIPS) $(LDFLAGS_LIBZ3) $(LDFLAGS_LIBCARL) $(LDFLAGS_TF)

  PLUGINS_all = $(PLUGINDIR)/clips-smt.$(SOEXT)

else
  ifneq ($(SUPPORT_CPP14),1)
    WARN_TARGETS += warning_cpp14
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS = warning_navgraph
  endif
  ifneq ($(HAVE_LIBZ3),1)
    WARN_TARGETS = warning_z3
  endif
  ifneq ($(HAVE_LIBCARL),1)
    WARN_TARGETS = warning_carl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS = warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips warning_cpp14 warning_z3 warning_carl warning_tf
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-smt plugin$(TNORMAL) ($(CLIPS_ERROR))"

warning_cpp14:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-smt plugin$(TNORMAL) (C++14 not supported)"

warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-smt plugin$(TNORMAL) (Navgraph not available)"

warning_z3:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-smt plugin$(TNORMAL) (Z3 Solver not available)"

warning_carl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-smt plugin$(TNORMAL) (Carl libraries not available)"

warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph plugin$(TNORMAL) (tf not available)"

endif

include $(BUILDSYSDIR)/base.mk
