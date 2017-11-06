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

CFLAGS   += $(CFLAGS_CPP11)

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,8),1)
    GCC_TOO_OLD=1
  endif
endif

PRESUBDIRS = interfaces

LIBS_protoboard = fawkescore fawkesaspects \
		fawkesblackboard fawkesinterface fawkesconfig \
    fawkes_protobuf_comm llsf_msgs \
		OrderInterface PrepareMachineInterface SendBeaconInterface ProtobufPeerInterface \
    RecvBeaconInterface RCLLGameStateInterface

OBJS_protoboard = blackboard_manager.o protobuf_thread.o interface_definitions.o plugin.o
 
ifneq ($(GCC_TOO_OLD),1)
  PLUGINS_all = $(PLUGINDIR)/protoboard.$(SOEXT)
else
  WARN_TARGETS += warning_old_gcc
endif

OBJS_all = $(OBJS_protoboard)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_old_gcc
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Can't build protoboard plugin$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.8)"
endif

include $(BUILDSYSDIR)/base.mk
