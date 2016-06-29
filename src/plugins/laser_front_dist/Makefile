#*****************************************************************************
#         Makefile Build System for Fawkes: laser_front_dist Plugin
#                            -------------------
#   Created on Thu Jun 23 22:35:32 2016
 
#   Copyright (C) 2016 by Frederik Zwilling
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

LIBS_laser_front_dist = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface Laser360Interface \
		    	Position3DInterface fawkestf

OBJS_laser_front_dist = laser_front_dist_plugin.o laser_front_dist_thread.o


OBJS_all = $(OBJS_laser_front_dist)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)

  PLUGINS_all = $(PLUGINDIR)/laser_front_dist.$(SOEXT)
else

  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_gazebo warning_protobuf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser-front-dist Plugin$(TNORMAL) " \
                         "(tf not available)"
endif


include $(BUILDSYSDIR)/base.mk
