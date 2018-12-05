
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

ifeq ($(CC),gcc)
  ifeq ($(call gcc_atleast_version,4,6),1)
    GCC_OK=1
  endif
endif

ifeq ($(HAVE_TF)$(GCC_OK),11)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  
  CFLAGS   += $(VISION_CFLAGS)
  LDFLAGS  += $(VISION_LDFLAGS)
  INCDIRS  += $(VISION_INCDIRS)
  LIBDIRS  += $(VISION_LIBDIRS)
  LIBS     += $(VISION_LIBS)

  #The necessary libraries your plugin needs.
  LIBS_puck_vision = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface fvutils fvcams fvmodels \
		fvclassifiers fawkesconfig fvfilters TransformInterface \
		fawkestf RobotinoLightInterface PuckVisionInterface Position3DInterface 
  #binaries that are needed for the given plugin (all your cpp-files, but with an o-ending :-) ).
  OBJS_puck_vision = puck_vision.o puck_vision_thread.o

  PLUGINS_all = $(PLUGINDIR)/puck_vision.$(SOEXT)

  OBJS_all = $(OBJS_puck_vision)

else
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(GCC_OK),1)
    WARN_TARGETS += warning_old_gcc
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_somewarning warning_old_gcc
warning_somewarning:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino Navigator plugin$(TNORMAL) (reason)"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS Plugins$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.6)"

endif

include $(BUILDSYSDIR)/base.mk
