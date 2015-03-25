#*****************************************************************************
#     Makefile Build System for Fawkes: generate navgraph for MPS game
#                            -------------------
#   Created on Mon Mar 23 16:05:51 2015
#   copyright (C) 2006-2015 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/eigen3.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk

PRESUBDIRS=interfaces

LIBS_navgraph_generator_mps = fawkescore fawkesutils fawkesaspects fawkesinterface \
		fawkesblackboard fawkestf fawkesnavgraph \
		Position3DInterface NavGraphGeneratorInterface \
		NavGraphWithMPSGeneratorInterface
OBJS_navgraph_generator_mps = navgraph_generator_mps_plugin.o navgraph_generator_mps_thread.o
OBJS_all    = $(OBJS_navgraph_generator_mps)

ifeq ($(HAVE_NAVGRAPH)$(HAVE_EIGEN3)$(HAVE_TF),111)
  CFLAGS  += $(CFLAGS_NAVGRAPH)  $(CFLAGS_EIGEN3)  $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_NAVGRAPH) $(LDFLAGS_EIGEN3) $(LDFLAGS_TF)

  PLUGINS_all = $(PLUGINDIR)/navgraph-generator-mps.$(SOEXT)
else
  # navgraph.mk warns already
  ifneq ($(HAVE_EIGEN3),1)
    WARN_TARGETS += warning_eigen
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_eigen
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-generator-mps plugin$(TNORMAL) (fawkestf not available)"
warning_eigen:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-generator-mps plugin$(TNORMAL) (Eigen3 not available)"
endif


include $(BUILDSYSDIR)/base.mk
