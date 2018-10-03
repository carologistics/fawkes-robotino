#*****************************************************************************
#          Makefile Build System for Fawkes: Webview for refbox logs
#                            -------------------
#   Created on Tue Mar 17 17:45:45 2015
#   Copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(FAWKES_BASEDIR)/src/plugins/mongodb/mongodb.mk
include $(BUILDCONFDIR)/webview/webview.mk

CFLAGS += $(CFLAGS_CPP11)

LIBS_webview_refbox = fawkescore fawkesutils fawkesaspects fawkeswebview \
		      fawkesmongodbaspect
OBJS_webview_refbox = webview-refbox-thread.o webview-refbox-processor.o \
		      webview-refbox-plugin.o

OBJS_all = $(OBJS_webview_refbox)

ifeq ($(HAVE_WEBVIEW)$(HAVE_MONGODB),11)
  CFLAGS  += $(CFLAGS_WEBVIEW)  $(CFLAGS_MONGODB)
  LDFLAGS += $(LDFLAGS_WEBVIEW) $(LDFLAGS_MONGODB)

  PLUGINS_all = $(PLUGINDIR)/webview-refbox.$(SOEXT)
else
  ifneq ($(HAVE_WEBVIEW),1)
    WARN_TARGETS += warning_webview
  endif
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_webview warning_mongodb
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webview-refbox plugin$(TNORMAL) (webview not available)"
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webview-refbox plugin$(TNORMAL) (MongoDB support not available)"
endif

include $(BUILDSYSDIR)/base.mk
