/***************************************************************************
 *  clips_smt_robot.h - Smt feature for CLIPS
 *
 *  Created: Created on Mon Dec 19 13::57 2016 by Igor Nicolai Bongartz
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

#ifndef _PLUGINS_CLIPS_SMT_ROBOT_H_
#define _PLUGINS_CLIPS_SMT_ROBOT_H_

#include "clips_smt_workingPiece.h"
#include "clips_smt_data.h"


using namespace std;

  /**
    Requirements from meeting:
      Current working piece for all robots [dyn]
      List of robots
      Number of robots and identity [sta2]

  */

class Robot {
private:
  unsigned int _id;
  WorkingPiece _wp;
  int _currentPosition;
  int _targetPosition;
  smtTime _busyTimeLeft;

public:
  //constructor
  Robot(unsigned int id, int currentPos, int targetPos, WorkingPiece wp)
  {
    _id = id;
    _currentPosition = currentPos;
    _targetPosition = targetPos;
    _wp = wp;
  }

  void setId(int id) 
  { 
    _id = id; 
  }
  
  unsigned int getId() const
  { 
    return _id; 
  }
  
  int getCurrentPosition() const
  {
    return _currentPosition;
  }
  
  int getTargetPosition() const
  {
    return _targetPosition;
  }

  smtTime getBusyTimeLeft() const
  {
    return _busyTimeLeft;
  }


};


#endif
