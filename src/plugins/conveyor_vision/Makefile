
#*****************************************************************************
#         Makefile Build System for Fawkes: Puck Vision
#                            -------------------
#   Created on Wed Jun 03 18:31:45 2013
#   Copyright (C) 2013 by Florian Nolden Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/fvconf.mk
include $(BUILDSYSDIR)/boost.mk

 
CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

# compile against and link against opencv
CFLAGS	 += $(CFLAGS_OPENCV)
LDFLAGS  += $(LDFLAGS_OPENCV)

LDFLAGS  += "-lopencv_core"
LDFLAGS  += "-lopencv_imgproc"
LDFLAGS  += "-lopencv_objdetect" 
LDFLAGS  += "-lopencv_highgui"

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
endif

#The necessary libraries your plugin needs.
LIBS_conveyor_vision = fawkescore fawkesutils fawkesaspects fvutils \
	        fawkestf fawkesinterface fawkesblackboard \
		   	Position3DInterface fvcams fvmodels fvclassifiers fvfilters \
            fawkesconfig 
			#binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
OBJS_conveyor_vision = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(foreach d,$(DRIVERS),$(SRCDIR)/$d/*.cpp )))))

PLUGINS_all = $(PLUGINDIR)/conveyor_vision.so
OBJS_all = $(OBJS_conveyor_vision)

ifeq ($(HAVE_TF),111)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  #names of the plugins you want to build
  PLUGINS_all = $(PLUGINDIR)/conveyor_vision.$(SOEXT)
else
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting conveyor-vision plugin$(TNORMAL) (tf not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting conveyor-vision plugin$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
