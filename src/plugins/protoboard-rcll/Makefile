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
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk

PRESUBDIRS = interfaces

LIBS_protoboard_rcll = fawkesprotoboard llsf_msgs \
  OrderInterface PrepareMachineInterface SendBeaconInterface ProtobufPeerInterface \
  RecvBeaconInterface RCLLGameStateInterface

CFLAGS += $(CFLAGS_CPP11)

OBJS_protoboard_rcll = rcll_receiving.o rcll_sending.o plugin.o
OBJS_all = $(OBJS_protoboard_rcll)

PLUGINS_all = $(PLUGINDIR)/protoboard-rcll.$(SOEXT) 
PLUGINS_build = $(PLUGINS_all)

include $(BUILDSYSDIR)/base.mk
