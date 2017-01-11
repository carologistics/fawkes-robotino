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


//#include "clips_smt_thread.h"
#include "clips_smt_workingPiece.h"
//#include "clips_smt_data.h"



/**
(Global and) local deadlines [dyn]
  Order
    time deadLine
    WorkingPiece targetPiece
*/

class Order {
  typedef float smtTime;
  //time format used for the SMT solving is defined here
private:
  	smtTime _deadline;
  	WorkingPiece _targetPiece;

public:
	Order(smtTime deadline, WorkingPiece tp)
	{
		_deadline = deadline;
		_targetPiece = tp;
	}

	smtTime getDeadline() const
	{
		return _deadline;
	}

	void setDeadline(smtTime deadline)
	{
		_deadline = deadline;
	}

	WorkingPiece getWorkingPiece() const
	{
		return _targetPiece;
	}

	void setWorkingPiece(WorkingPiece targetPiece)
	{
		_targetPiece = targetPiece;
	}

  std::string toString()
  {
    std::string orderDescription;
    orderDescription += "Order has deadline ";
    orderDescription += std::to_string(_deadline);
    orderDescription += " and requires workingPiece ";
    orderDescription += _targetPiece.toString();
    return orderDescription;
  }
};

#endif
