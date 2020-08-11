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

PRESUBDIRS = interfaces 

LIBS_picture_taker = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkesinterface fawkesblackboard \
	       PictureTakerInterface hdf5
OBJS_picture_taker = picture_taker_plugin.o picture_taker_thread.o

OBJS_all    = $(OBJS_picture_taker)

CFLAGS  += $(CFLAGS_TF)  -DIS_SLIM_BUILD -std=c++11
LDFLAGS += $(LDFLAGS_TF)  -lstdc++ -lm -lz  -ldl

PLUGINS_all = $(PLUGINDIR)/picture_taker.$(SOEXT)

  LDFLAGS  += "-lopencv_core"
  LDFLAGS  += "-lopencv_imgproc"
  LDFLAGS  += "-lopencv_objdetect"
  LDFLAGS  += "-lopencv_highgui"

PLUGINS_build += $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
