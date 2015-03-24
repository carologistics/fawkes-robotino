
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

# link against alvar
LDFLAGS  += "-lalvar200"

#The necessary libraries your plugin needs.
LIBS_tag_vision = fawkescore fawkesutils fawkesaspects fvutils \
	        fawkestf fawkesinterface fawkesblackboard \
		   	Position3DInterface fvcams fvmodels fvclassifiers fvfilters \
            fawkesconfig TagVisionInterface
			#binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
OBJS_tag_vision = tag_vision_plugin.o tag_vision_thread.o tag_position_list.o tag_position_interface_helper.o


OBJS_all = $(OBJS_tag_vision)

ifneq ($(wildcard $(SYSROOT)/usr/lib64/libalvar200.so $(SYSROOT)/usr/include/alvar),)
  HAVE_ALVAR = 1
endif

ifeq ($(HAVE_TF)$(HAVE_ALVAR)$(HAVE_CPP11),111)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  #names of the plugins you want to build
  PLUGINS_all = $(PLUGINDIR)/tag_vision.$(SOEXT)
else
  ifneq ($(HAVE_ALVAR),1)
    WARN_TARGETS += warning_alvar
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_alvar warning_tf warning_cpp11
warning_alvar:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting tag-vision plugin$(TNORMAL) (ALVAR not available)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting tag-vision plugin$(TNORMAL) (tf not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting tag-vision plugin$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
