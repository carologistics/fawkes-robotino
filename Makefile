#*****************************************************************************
#           Makefile Build System for Fawkes: RoboCup Logistics repo
#                            -------------------
#   Created on Mon May 14 11:39:00 2012
#   Copyright (C) 2012 by Tim Niemueller, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = .

SUBDIRS = fawkes src

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/rules.mk
include $(BUILDSYSDIR)/root/root.mk
include $(BUILDSYSDIR)/root/linkscripts.mk

src: fawkes

