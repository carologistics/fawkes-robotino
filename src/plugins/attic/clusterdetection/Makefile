#*****************************************************************************
#         Makefile Build System for Fawkes: laserclusterdetector plugin
#                            -------------------
#   Created on Wed May 23 18:31:45 2012
#   Copyright (C) 2012 by Daniel Ewert, Carologistics RoboCup Team
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

LIBS_laserclusterdetector = m fawkescore fawkesutils fawkestf fawkesaspects \
                            fawkesbaseapp fawkesblackboard fawkesinterface \
                            Laser360Interface TransformInterface Position3DInterface
OBJS_laserclusterdetector = laser_cluster_detector_plugin.o laser_cluster_detector_thread.o

OBJS_all    = $(OBJS_laserclusterdetector)
PLUGINS_all = $(PLUGINDIR)/laserclusterdetector.$(SOEXT)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)

  PLUGINS_build = $(PLUGINS_all)
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino tf plugin$(TNORMAL) (fawkestf not available)"
endif

include $(BUILDSYSDIR)/base.mk
