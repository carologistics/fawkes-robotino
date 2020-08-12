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

SUBDIRS	= clips-motor-switch \
	  voltagelogger \
	  ax12_gripper \
	  arduino \
		tag_vision \
      navgraph_broker \
	  navgraph-generator-mps \
	  conveyor_pose \
	  conveyor_plane \
	  gazebo \
	  marker_writer \
		laser_front_dist \
		mps-laser-gen \
		webtools-bridge \
		skiller_motor_state \
		protoboard-rcll \
		asp-planner \
    yolo_opencv \
    workpiece_pose

include $(BUILDSYSDIR)/rules.mk

gazebo: ax12_gripper tag_vision arduino conveyor_pose navgraph-generator-mps
