/***************************************************************************
 *  asp_planner_externals.h - ASP-based planner externals
 *
 *  Created on Fri Dec 16 17:00:02 2016
 *  Copyright (C) 2016 by Björn Schäpers
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_ASP_PLANNER_EXTERNALS_H_
#define __PLUGINS_ASP_PLANNER_EXTERNALS_H_

#include <string>

namespace Clingo {
class Symbol;
}

struct ProductIdentifier;
struct TaskDescription;

Clingo::Symbol generateAliveExternal(const std::string &robotName);
Clingo::Symbol generateRobotLocationExternal(const std::string &   robotName,
                                             const Clingo::Symbol &location);
Clingo::Symbol generateHoldingExternal(const std::string &      robotName,
                                       const ProductIdentifier &product);
Clingo::Symbol generateDoingExternal(const std::string &    robotName,
                                     const TaskDescription &task,
                                     const int              duration);

Clingo::Symbol generateExploreLocationExternal(const int zone);
Clingo::Symbol generateExploreTaskExternal(const int zone);

Clingo::Symbol generateProductExternal(const ProductIdentifier &product);
Clingo::Symbol generateProductBaseExternal(const ProductIdentifier &product,
                                           const std::string &      base);
Clingo::Symbol generateProductRingExternal(const ProductIdentifier &product,
                                           const int                ringNumber,
                                           const std::string &      color);
Clingo::Symbol generateProductCapExternal(const ProductIdentifier &product, const std::string &cap);

Clingo::Symbol generateMachineBrokenExternal(const std::string &machineName, const int duration);
Clingo::Symbol generateMachineWorkingExternal(const std::string &      machineName,
                                              const int                duration,
                                              const ProductIdentifier &product);
Clingo::Symbol generateMachineStoringExternal(const std::string &      machineName,
                                              const ProductIdentifier &product);
Clingo::Symbol generatePreparedExternal(const std::string &machineName);
Clingo::Symbol generateFillStateExternal(const std::string &machineName, const int fillState);

Clingo::Symbol
generateLocationExternal(const char *teamColor, const char *machine, const char *side);

Clingo::Symbol generateMountRingExternal(const Clingo::Symbol &location,
                                         const int             orderNumber,
                                         const int             qty,
                                         const int             ring);
Clingo::Symbol
generateMountCapExternal(const Clingo::Symbol &location, const int orderNumber, const int qty);
Clingo::Symbol
generateDeliverExternal(const Clingo::Symbol &location, const int orderNumber, const int qty);
Clingo::Symbol
generateLateDeliverExternal(const Clingo::Symbol &location, const int orderNumber, const int qty);

#endif
