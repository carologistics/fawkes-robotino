#*****************************************************************************
#               Makefile Build System for Fawkes: Genesis Plugin
#
#   Created on Sun Nov 13 15:19:32 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
##*****************************************************************************
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

PRESUBDIRS = interfaces 

LIBS_tensorflow = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard \
	       TensorflowInterface
OBJS_tensorflow = tensorflow_plugin.o tensorflow_thread.o

OBJS_all    = $(OBJS_tensorflow)

CFLAGS  += $(CFLAGS_TF)  -DIS_SLIM_BUILD -std=c++11
LDFLAGS += $(LDFLAGS_TF)  -ltensorflow -lstdc++ -lm -lz  -ldl

PLUGINS_all = $(PLUGINDIR)/tensorflow.$(SOEXT)



ifeq ($(OBJSUBMAKE),1)
	ifneq($(WARN_TARGETS),)
all: $(WARN_TARGETS)
	endif




include $(BUILDSYSDIR)/base.mk
