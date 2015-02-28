
#*****************************************************************************
#            Makefile Build System for Fawkes: ax12-gripper Plugin
#                            -------------------
#   Created on Sat Feb 28 10:27:16 2015
#   Copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
#                      2015    Nicolas Limpert
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
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces
#SUBDIRS = tool

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
endif

# Sub directory name that contain Robotis drivers
DRIVERS = robotis

LIBS_ax12_gripper = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	       fawkesinterface AX12GripperInterface LedInterface JointInterface
OBJS_ax12_gripper = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(foreach d,$(DRIVERS),$(SRCDIR)/$d/*.cpp )))))

PLUGINS_all = $(PLUGINDIR)/ax12_gripper.so
OBJS_all = $(OBJS_ax12_gripper)

include $(BUILDSYSDIR)/base.mk

