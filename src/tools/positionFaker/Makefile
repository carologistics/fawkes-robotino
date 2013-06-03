#*****************************************************************************
#        Makefile Build System for Fawkes : flitePromt
#                            -------------------
#   Created on Sat Mar 15 13:54:37 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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
#include $(BASEDIR)/src/plugins/flite/flite.mk

LIBS_3dpositionFaker = stdc++ fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
		fawkesinterface \
		readline history $(if $(HAVE_TERMCAP),termcap) \
		Position3DInterface
OBJS_3dpositionFaker = 3dpositionFaker.o

OBJS_all =	$(OBJS_3dpositionFaker)

ifeq (1,1)
  BINS_all =	$(BINDIR)/3dpositionFaker
  MANPAGES_all = $(MANDIR)/man1/3dpositionFaker.1
else
  WARN_TARGETS += warning_readline
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_readline
warning_readline:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting 3dpositionFaker console tool$(TNORMAL) (readline[-devel] not installed)"
endif


include $(BUILDSYSDIR)/base.mk

