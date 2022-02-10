#*****************************************************************************
#         Makefile Build System for Fawkes: Object Tracking Plugin
#                            -------------------
#   Created on Tue Jan 25 18:25:15 2022
#   Copyright (C) 2022 by Matteo Tschesche, Carologistics RoboCup Team
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
include $(BUILDCONFDIR)/navgraph/navgraph.mk
include $(BUILDCONFDIR)/tf/tf.mk

CFLAGS   += $(VISION_CFLAGS) $(CFLAGS_NAVGRAPH) $(CFLAGS_TF) $(CFLAGS_OPENCV)
LDFLAGS  += $(VISION_LDFLAGS) $(LDFLAGS_NAVGRAPH) $(LDFLAGS_TF) $(LDFLAGS_OPENCV)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_object_tracking = m fawkescore fawkesutils fawkesaspects \
    fawkesinterface fawkesblackboard ObjectTrackingInterface \
    fvutils
OBJS_object_tracking = object_tracking_plugin.o object_tracking_thread.o

OBJS_all    = $(OBJS_object_tracking)

PLUGINS_all = $(PLUGINDIR)/object-tracking.$(SOEXT)

OPENCV_MIN_VERSION = 4.5.0

ifeq ($(shell $(PKGCONFIG) --exists opencv; echo $$?),0)
    ifeq ($(shell $(PKGCONFIG) --atleast-version $(OPENCV_MIN_VERSION) opencv; echo $$?),0)
        CFLAGS  += $(shell $(PKGCONFIG) --cflags opencv) -Wno-error
    LDFLAGS += $(shell $(PKGCONFIG) --libs opencv)
        PLUGINS_build = $(PLUGINS_all)
    else
        WARN_TARGETS += warning_opencv_version
        OPENCV_VERSION = $(shell $(PKGCONFIG) --modversion opencv)
    endif
else
    WARN_TARGETS += warning_opencv
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_opencv warning_opencv_version
warning_opencv:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Yolo OpenCV Plugin$(TNORMAL) (opencv not found)"
warning_opencv_version:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Yolo OpenCV Plugin$(TNORMAL) (incompatible opencv version $(OPENCV_VERSION); required >= $(OPENCV_MIN_VERSION))"
endif

include $(BUILDSYSDIR)/base.mk
