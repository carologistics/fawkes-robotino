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
  int id;
  int positionA;
  int positionB;
  WorkingPiece wp;
  //time defaultBusyTime
  //time busyTimeLeft
  MachineType machineType; // TODO from Igor: Or maybe Vector<int> outputRequirement
};

#endif
