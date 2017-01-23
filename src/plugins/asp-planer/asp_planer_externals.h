/***************************************************************************
 *  asp_planer_externals.h - ASP-based planer externals
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

#ifndef __PLUGINS_ASP_PLANER_EXTERNALS_H_
#define __PLUGINS_ASP_PLANER_EXTERNALS_H_

#include <string>

namespace Clingo {
class Symbol;
}

class TaskDescription;

struct ProductIdentifier;

Clingo::Symbol generateAliveExternal(const std::string& robotName);
Clingo::Symbol generateRobotLocationExternal(const std::string& robotName, const Clingo::Symbol& location);
Clingo::Symbol generateHoldingExternal(const std::string& robotName, const ProductIdentifier& product);
Clingo::Symbol generateDoingExternal(const std::string& robotName, const TaskDescription& task);

Clingo::Symbol generateExploreLocationExternal(const unsigned int zone);
Clingo::Symbol generateExploreTaskExternal(const unsigned int zone);

Clingo::Symbol generateProductExternal(const int id);
Clingo::Symbol generateProductBaseExternal(const int id, const std::string& base);
Clingo::Symbol generateProductRingExternal(const int id, const int ringNumber, const std::string& color);
Clingo::Symbol generateProductCapExternal(const int id, const std::string& cap);

#endif
