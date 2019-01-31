
#*****************************************************************************
#         Makefile Build System for Fawkes: Robotino Motor LED Indicator
#                            -------------------
#   Created on Fri Apr 15 15:00:36 2016
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_motor_led = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                 fawkesblackboard fawkesinterface \
                 MotorInterface RobotinoSensorInterface
OBJS_motor_led = motor_led_plugin.o motor_led_thread.o

CFLAGS += $(CFLAGS_CPP11)

OBJS_all    = $(OBJS_motor_led)
PLUGINS_all = $(PLUGINDIR)/motor-led.$(SOEXT)
PLUGINS_build = $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
