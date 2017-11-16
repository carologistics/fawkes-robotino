#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Thu 16 Nov 2017 10:47:37 CET
#   Copyright (C) 2017 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

LIBS_configure_shelf = stdc++ m fawkescore fawkesutils fawkesblackboard \
											 fawkesnetcomm fawkesinterface ShelfConfigurationInterface

OBJS_configure_shelf = configure_shelf.o
OBJS_all = $(OBJS_configure_shelf)

BINS_all += $(BINDIR)/configure_shelf

include $(BUILDSYSDIR)/base.mk
