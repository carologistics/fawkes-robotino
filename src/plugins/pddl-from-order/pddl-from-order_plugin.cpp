
/***************************************************************************
 *  pddl-from-order_plugin.cpp - pddl-from-order
 *
 *  Created: Sat Mar 18 19:46:59 2017
 *  Copyright  2017  Matthias Loebach
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

#include <core/plugin.h>

#include "pddl-from-order_thread.h"

using namespace fawkes;

/** @class PddlFromOrderPlugin "pddl-from-order_plugin.cpp"
 * Generates a PDDL problem from an incoming RCLL order
 * @author Matthias Loebach
 */
class PddlFromOrderPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fakwes configuration
   */
  PddlFromOrderPlugin(Configuration *config)
     : Plugin(config)
  {
     thread_list.push_back(new PddlFromOrderThread());
  }
};

PLUGIN_DESCRIPTION("Generates a PDDL problem from an incoming RCLL order")
EXPORT_PLUGIN(PddlFromOrderPlugin)