/***************************************************************************
 *  clips_smt_order.h - Smt feature for CLIPS
 *
 *  Created: Created on Mon Dec 19 13::59 2016 by Igor Nicolai Bongartz
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

#ifndef _PLUGINS_CLIPS_SMT_ORDER_H_
#define _PLUGINS_CLIPS_SMT_ORDER_H_

#include "clips_smt_workingPiece.h"

/**
(Global and) local deadlines [dyn]
  Order
    time deadLine
    WorkingPiece targetPiece
*/

class Order {
private:
  //time deadLine
  WorkingPiece target;
};

#endif
