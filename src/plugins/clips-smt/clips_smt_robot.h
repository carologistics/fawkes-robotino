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
//#include "clips_smt_data.h"


using namespace std;

  /**
    Requirements from meeting:
      Current working piece for all robots [dyn]
      List of robots
      Number of robots and identity [sta2]

  */

class Robot {
  typedef float smtTime;
  //time format used for the SMT solving is defined here
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

  unsigned int getId() const
  {
    return _id;
  }

  int getCurrentPosition() const
  {
    return _currentPosition;
  }

  void setCurrentPosition(int currentPos)
  {
    _currentPosition = currentPos;
  }

  int getTargetPosition() const
  {
    return _targetPosition;
  }

  void setTargetPosition(int targetPos)
  {
    _targetPosition = targetPos;
  }

  smtTime getBusyTimeLeft() const
  {
    return _busyTimeLeft;
  }

  void setBusyTimeLeft(smtTime busyTimeLeft)
  {
    _busyTimeLeft = busyTimeLeft;
  }

  std::string toString()
  {
    std::string robotDescription;
    robotDescription += "Robot [id:";
    robotDescription += std::to_string(_id);
    robotDescription += "] is at position ";
    robotDescription += std::to_string(_currentPosition);
    robotDescription += " -> ";
    robotDescription += std::to_string(_targetPosition);
    robotDescription += " and holds workingPiece ";
    robotDescription += _wp.toString();
    return robotDescription;
  }
};


#endif
