#*****************************************************************************
#               Makefile Build System for Fawkes: Bridge Plugin
#
#   Created on Thu Sep 27 14:23:00 2012
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/clips.mk

LIBS_bridge = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard \
	       Position3DInterface fawkesclipsaspect RobotinoLightInterface SkillerInterface
OBJS_bridge = bridge_plugin.o bridge_thread.o \
				bridge_processor.o bridge_interface.o ros_proxy.o

OBJS_all    = $(OBJS_bridge)



#======temp:: TO BE REPLACED with python.mk and BOOST.mk includes
PYTHON_VERSION = 2.7
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
 
# location of the Boost Python include files and library
 
BOOST_INC = /usr/include
BOOST_LIB = /usr/lib

MY_CFLAGS = -I$(PYTHON_INCLUDE) -I$(BOOST_INC)
MY_LDFLAGS = -L$(BOOST_LIB) -lboost_python -L/usr/lib/python$(PYTHON_VERSION)/config -lpython$(PYTHON_VERSION)


# TARGET = bridge_interface
#======EndTemp



ifeq ($(HAVE_TF)$(HAVE_CLIPS)$(HAVE_CPP11),111)
  CFLAGS  += $(CFLAGS_TF) $(CFLAGS_CLIPS) $(CFLAGS_CPP11) $(MY_CFLAGS)
  LDFLAGS += $(LDFLAGS_TF) $(MY_LDFLAGS)

  PLUGINS_all = $(PLUGINDIR)/bridge.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_ros warning_geometry_msgs warning_cpp11 warning_clips
warning_tf: 
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting bridge libs" \
                         "$(TNORMAL) (tf not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build bridge plugin$(TNORMAL) (C++11 not supported)"
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build bridge plugin$(TNORMAL) ($(CLIPS_ERROR))"
endif


##===>Stupid temp..find another way to do it
#$(TARGET).so: .objs_fawkes/$(TARGET).o

include $(BUILDSYSDIR)/base.mk


#$(TARGET).so: .objs_fawkes/$(TARGET).o
	# g++ -shared -Wl,--export-dynamic .objs_fawkes/$(TARGET).o $(LDFLAGS) -o $(TARGET).so


