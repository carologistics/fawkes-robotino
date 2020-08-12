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

PRESUBDIRS = interfaces 

LIBS_yolo_opencv = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkesinterface fawkesblackboard \
	       YoloOpenCVInterface
OBJS_yolo_opencv = yolo_opencv_plugin.o yolo_opencv_thread.o

OBJS_all    = $(OBJS_yolo_opencv)

CFLAGS  += $(CFLAGS_TF)  -DIS_SLIM_BUILD -std=c++11
LDFLAGS += $(LDFLAGS_TF)  -lstdc++ -lm -lz  -ldl

PLUGINS_all = $(PLUGINDIR)/yolo_opencv.$(SOEXT)

  LDFLAGS  += "-lopencv_imgproc"
  LDFLAGS  += "-lopencv_dnn"
  LDFLAGS  += "-lopencv_highgui"

PLUGINS_build += $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
