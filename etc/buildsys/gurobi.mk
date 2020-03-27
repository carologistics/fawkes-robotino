#*****************************************************************************
#                Makefile Build System for Fawkes: CLIPS bits
#                            -------------------
#   Created on Fri Mar 27 14:49:41 2012 (Haus Quranatine, Aachen)
#   Copyright (C) 2020 by Mostafa Gomaa, RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before gurobi.mk)
endif

ifndef __buildsys_gurobi_mk_
__buildsys_gurobi_mk_ := 1

GUROBI_ERROR=

__GUROBI_INCLUDE_PATHS=/usr/include /usr/local/include $(GUROBI_DIR)/include

HAVE_GUROBI = $(if $(wildcard $(addsuffix  /gurobi_c++.h,$(__GUROBI_INCLUDE_PATHS))),1)


ifeq ($(HAVE_GUROBI),1)
    CFLAGS_GUROBI += -I$(GUROBI_DIR)/include/
    LDFLAGS_GUROBI += -L$(GUROBI_DIR)/lib -lgurobi_c++ -lgurobi90
else
    GUROBI_ERROR = GUROBI not found
endif

endif # __buildsys_gurobi_mk_
