/***************************************************************************
 *  clips_smt_data.h - Smt feature for CLIPS
 *
 *  Created: Created on Mon Dec 19 11:28 2016 by Igor Nicolai Bongartz
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

#ifndef _PLUGINS_CLIPS_SMT_DATA_H_
#define _PLUGINS_CLIPS_SMT_DATA_H_

#include "clips_smt_robot.h"
#include "clips_smt_machine.h"
#include "clips_smt_order.h"
#include "clips_smt_thread.h"


struct clips_smt_data {
  /**
    Requirements from meeting:
      Current order [dyn]
      Points for working step [sta1] TODO from Igor: What are pointsForWorkingStep exactly
      Navgraph [sta2]
      Which steps are necessary for order [sta2]
      NavGraph navGraph (??) TODO from Igor: How to access a navGraph object

      Robots position and length of current action?
  */
  std::vector<Robot> robots;
  std::vector<Machine> machines;
  std::vector<Order> currentOrders;
  //pointsForWorkingStep
  //NavGraph
};
#endif
