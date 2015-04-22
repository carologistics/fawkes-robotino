#*****************************************************************************
#                  Makefile Build System for Fawkes: Plugins
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

BASEDIR = ../..
include $(BASEDIR)/etc/buildsys/config.mk

SUBDIRS	= omnivision_pucks clips-motor-switch omnivision_activator \
	  light_front robotino_worldmodel voltagelogger machine-signal puck_vision \
	  gazebo agent-monitor marker_writer ax12_gripper tag_vision \
      navgraph_broker navgraph-generator-mps \
	  gazebo agent-monitor marker_writer \

# navgraph_broker

include $(BUILDSYSDIR)/rules.mk

gazebo: ax12_gripper tag_vision
