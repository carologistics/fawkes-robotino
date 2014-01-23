#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-llsf-statistics Plugin
#
#   Created on Mon Sep 23 17:11:35 2013
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#                         Frederik Zwilling
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
include $(BASEDIR)/fawkes/src/plugins/gazebo/gazebo.mk
include $(BASEDIR)/fawkes/src/plugins/gazebo/protobuf.mk
include $(BASEDIR)/fawkes/src/plugins/mongodb/mongodb.mk

LIBS_gazsim_llsf_statistics = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard protobufmsgs\
	       Position3DInterface TransformInterface \
	       SwitchInterface RobotinoLightInterface \
	       fawkesgazeboaspect fawkesmongodbaspect
OBJS_gazsim_llsf_statistics = gazsim_llsf_statistics_plugin.o gazsim_llsf_statistics_thread.o

OBJS_all    = $(OBJS_gazsim_llsf_statistics)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_TF)$(HAVE_MONGODB),1111)
  CFLAGS  += $(CFLAGS_TF) $(CFLAGS_MONGODB)
  LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_MONGODB)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system) -lboost_system

  PLUGINS_all = $(PLUGINDIR)/gazsim-llsf-statistics.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_mongodb warning_gazebo warning_protobuf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-llsf-statistics Plugin$(TNORMAL) " \
                         "(tf not available)"
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-llsf-statistics Plugin$(TNORMAL) " \
		"(mongodb[-devel] not installed)"
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-llsf-statistics Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-llsf-statistics Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
