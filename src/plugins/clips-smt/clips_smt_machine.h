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
//#include "clips_smt_data.h"

  /**
    Requirements from meeting:
      Current working piece for all machines [dyn]
      List of machines
      Size of aux-materials-queue [dyn]
      Lenght of working step of a machine [sta2]
  */

enum MachineType {base, cap, ring, delivery};

class Machine {
  typedef float smtTime;
  //time format used for the SMT solving is defined here
private:
  unsigned int _id;
  int _positionA;
  int _positionB;
  WorkingPiece _wp;
  std::vector<WorkingPieceComponent> _inputWpType;
  std::vector<WorkingPieceComponent> _inputWpContainer;
  WorkingPieceComponent _outputWpType;
  smtTime _defaultBusyTime;
  smtTime _busyTimeLeft;
  MachineType _machineType; // TODO from Igor: Or maybe Vector<int> outputRequirement

public:

  //Constructor
  Machine(unsigned int id, int posA, int posB, smtTime defaultBusyTime, MachineType machineType, WorkingPiece wp,   
          std::vector<WorkingPieceComponent> inputWpType,
          std::vector<WorkingPieceComponent> inputWpContainer,
          WorkingPieceComponent outputWpType)
  {
    _id = id;
    _positionA = posA;
    _positionB = posB;
    _defaultBusyTime = defaultBusyTime;
    _machineType = machineType;
    if (!wp.isConsistent()) throw std::runtime_error("SMT_ERROR: Cannot Set Inconsistent WorkingPiece for Machine");
    _wp = wp;

    _inputWpType = inputWpType;
    _inputWpContainer = inputWpContainer;
    _outputWpType = outputWpType;
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
    if (!wp.isConsistent()) throw std::runtime_error("SMT_ERROR: Cannot Set Inconsistent WorkingPiece for Machine");
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

  WorkingPieceComponent getOutputType() const
  {
    return _outputWpType;
  }

  std::vector<WorkingPieceComponent> getInputType() const
  {
    return _inputWpType;
  }

  std::vector<WorkingPieceComponent> getInputContainer() const
  {
    return _inputWpContainer;
  }

  bool hasRecievedWorkPieceComponent(WorkingPieceComponent wpc) const
  {
    for(WorkingPieceComponent workingPieceComponent: _inputWpContainer) {
      if (wpc == workingPieceComponent) return true;
    }

    return false;
  }


  bool hasRecievedWorkPieceComponent(int input) const
  {
    if (input < 0 ) throw std::runtime_error("SMT_ERROR: Unable to Convert negative Number to WorkingPieceComponent.");
    WorkingPieceComponent wpc = static_cast<WorkingPieceComponent>(input);
    return hasRecievedWorkPieceComponent(wpc);
  }


  std::string toString()
  {
    std::string machineDescription;
    machineDescription += "Machine [id:";
    machineDescription += std::to_string(_id);
    machineDescription += ", machineType:";
    machineDescription += std::to_string(_machineType);
    machineDescription += "] is at positions [";
    machineDescription += std::to_string(_positionA);
    machineDescription += ", ";
    machineDescription += std::to_string(_positionB);
    machineDescription += "] and holds workingPiece ";
    machineDescription += _wp.toString();
    return machineDescription;
  }
};


#endif
