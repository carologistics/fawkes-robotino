#*****************************************************************************
#             Makefile Build System for Fawkes : ASP Protobuf Messages
#                            -------------------
#   Created on Mon Aug 15 16:42:33 2016
#   Copyright (C) 2016 by Björn Schäpers
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

PROTOBUF_all = asp_msgs

MSGS_asp_msgs = $(notdir $(patsubst %.proto,%,$(wildcard $(SRCDIR)/*.proto)))

include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/base.mk
