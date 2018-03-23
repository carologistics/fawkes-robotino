#*****************************************************************************
#               Makefile Build System for Fawkes: Velocity Share Plugin
#
#   Created on Sat Jan 20 16:25:00 2018
#   Copyright (C) 2018 by Nicolas Limpert
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
include $(FAWKES_BASEDIR)/src/plugins/mongodb/mongodb.mk
include $(FAWKES_BASEDIR)/src/plugins/robot-memory/robot_memory.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/ros.mk


LIBS_velocity_share = fawkescore fawkesutils fawkesaspects \
                      fawkesinterface fawkesblackboard fawkestf \
                      fawkesconfig fawkesrobotmemory

OBJS_velocity_share = velocity_share_plugin.o velocity_share_thread.o

OBJS_all = $(OBJS_velocity_share)

ifeq ($(HAVE_ROBOT_MEMORY)$(HAVE_ROS)$(HAVE_TF),111)
  ifeq ($(call ros-have-pkg,velocity_share_msgs),1)

    CFLAGS  += $(CFLAGS_TF) $(CFLAGS_ROS) $(call ros-pkg-cflags,velocity_share_msgs)
    LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_ROS) $(call ros-pkg-lflags,velocity_share_msgs)
  
    PLUGINS_all = $(PLUGINDIR)/velocity_share.$(SOEXT)
  else
    WARN_TARGETS+= warning_velocity_msgs
  endif
else
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS = warning_tf
  endif
  ifneq ($(HAVE_ROBOT_MEMORY),1)
    WARN_TARGETS = warning_robot_memory
  endif
  ifneq ($(HAVE_ROS),1)
    WARN_TARGETS = warning_ros
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_ros warning_tf warning_robot_memory
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting velocity_share plugin" \
			"$(TNORMAL) (ros not available)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting velocity_share plugin" \
			"$(TNORMAL) (tf not available)"
warning_robot_memory:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting velocity_share plugin" \
			"$(TNORMAL) (robot memory not available)"
warning_velocity_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting velocity_share plugin" \
			"$(TNORMAL) (velocity_share_msgs ROS package not available)"

endif

include $(BUILDSYSDIR)/base.mk
