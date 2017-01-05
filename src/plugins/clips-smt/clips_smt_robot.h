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

using namespace std;

  /**
    Requirements from meeting:
      Current working piece for all robots [dyn]
      List of robots
      Number of robots and identity [sta2]
  */

class Robot {
private:
  int id;
  WorkingPiece wp;
  int currentPosition;
  int targetPosition;
  float busyTime;
public:
  void setId(int given_id) { id = given_id; }
  int getId() { return id; }
};


#endif
