/***************************************************************************
 *  clips_smt_machine.h - Smt feature for CLIPS
 *
 *  Created: Created on Mon Dec 19 13::58 2016 by Igor Nicolai Bongartz
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

#ifndef _PLUGINS_CLIPS_SMT_MACHINE_H_
#define _PLUGINS_CLIPS_SMT_MACHINE__H_

#include "clips_smt_workingPiece.h"
#include "clips_smt_data.h"

  /**
    Requirements from meeting:
      Current working piece for all machines [dyn]
      List of machines 
      Size of aux-materials-queue [dyn]
      Lenght of working step of a machine [sta2]
  */

enum MachineType {base, cap, ring, delivery};

class Machine {

private:
  unsigned int _id;
  int _positionA;
  int _positionB;
  WorkingPiece _wp;
  smtTime _defaultBusyTime
  smtTime _busyTimeLeft
  MachineType _machineType; // TODO from Igor: Or maybe Vector<int> outputRequirement

public:

  //Constructor
  Machine(unsigned int id, int posA, int posB, smtTime defaultBusyTime, MachineType machineType)
  {
    _id = id;
    _positionA = posA;
    _positionB = posB;
    _defaultBusyTime = defaultBusyTime;
    _machineType = machineType;
  }

  //getter & setters
  unsigned int getId() const
  {
    return _id;
  }

  int getPositionA() const
  {
    return _positionA;
  }

  int getPositionB() const
  {
    return _positionB;
  }
  
  WorkingPiece getWorkingPiece() const
  {
    return _wp;
  }
  
  void setWorkingPiece(WorkingPiece wp)
  {
    //TODO (Lukas) add working piece check for wp here
    _wp = wp;
  }

  void setBusyTimeLeft( smtTime timeLeft)
  {
    _busyTimeLeft = timeLeft;
  }

  smtTime getBusyTimeLeft() const
  {
    return _busyTimeLeft;
  }

  MachineType getMachineType() const
  {
    return _machineType;
  }

};


#endif
