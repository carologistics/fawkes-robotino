#*****************************************************************************
#               Makefile Build System for Fawkes: Yolo OpenCV Plugin
#
#   Created on Wed Aug 12 11:40:00 2020
#   Copyright (C) 2020 by Niklas Sebastian Eltester
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

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

CFLAGS   += $(CFLAGS_OPENCV)
LDFLAGS  += $(LDFLAGS_OPENCV)

PRESUBDIRS = interfaces 

LIBS_yolo_opencv = m fawkescore fawkesutils fawkesaspects \
	       fawkesinterface fawkesblackboard \
	       YoloOpenCVInterface
OBJS_yolo_opencv = yolo_opencv_plugin.o yolo_opencv_thread.o

OBJS_all    = $(OBJS_yolo_opencv)

PLUGINS_all = $(PLUGINDIR)/yolo_opencv.$(SOEXT)

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
