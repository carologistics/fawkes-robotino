#*****************************************************************************
#               Makefile Build System for Fawkes: Genesis Plugin
#
#   Created on Thu May 7 09:40:00 2017
#   Copyright (C) 2017 by Sebastian Schoenitz, Daniel Habering, 
#   			  Carsten Stoffels
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

LIBS_markerless = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard \
	       MPSRecognitionInterface hdf5
OBJS_markerless = markerless_rec_plugin.o markerless_rec_thread.o

OBJS_all    = $(OBJS_markerless)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)  -DIS_SLIM_BUILD -std=c++11
  LDFLAGS += $(LDFLAGS_TF)  -L/home/Sagre/tensorflow/bazel-bin/tensorflow -ltensorflow -lstdc++ -lm -lz  -ldl

  PLUGINS_all = $(PLUGINDIR)/markerless.$(SOEXT)

  LDFLAGS  += "-lopencv_core"
  LDFLAGS  += "-lopencv_imgproc"
  LDFLAGS  += "-lopencv_objdetect"
  LDFLAGS  += "-lopencv_highgui"

else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_ros warning_geometry_msgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting markerless_recognition libs" \
                         "$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
