#*****************************************************************************
#         Makefile Build System for Fawkes: pddl-from-order Plugin
#                            -------------------
#   Created on Sat Mar 18 19:46:59 2017
 
#   Copyright (C) 2017 by Matthias Loebach
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

LIBS_pddl_from_order = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface PddlPlannerInterface \
											PddlGenInterface

OBJS_pddl_from_order = pddl-from-order_plugin.o pddl-from-order_thread.o

PLUGINS_all = $(PLUGINDIR)/pddl-from-order.$(SOEXT)

OBJS_all = $(OBJS_pddl-from-order)

include $(BUILDSYSDIR)/base.mk
