#*****************************************************************************
#                  Makefile Build System for Fawkes: plugins/gazebo
#                            -------------------
#   Created on Mon May 14 11:37:00 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, Carologistics RoboCup Team
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

PRESUBDIRS = msgs

SUBDIRS	= gazsim-llsfrbcomm \
          gazsim-llsf-control \
          gazsim-gripper \
          gazsim-conveyor gazsim-tag-vision \
          gazsim-navgraph-generator \
          gazsim-object-tracking

include $(BUILDSYSDIR)/rules.mk
