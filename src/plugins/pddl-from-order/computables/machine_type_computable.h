/***************************************************************************
 *  machine_type_computable.h - Computable for retrieving a full machine_type
 *    
 *
 *  Created: Thu 01 Jun 2017 22:48:37 CEST
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

#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_MACHINE_TYPE_COMPUTABLE_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_MACHINE_TYPE_COMPUTABLE_H_

#include <aspect/logging.h>
#include <config/config.h>
#include <plugins/robot-memory/robot_memory.h>


class MachineTypeComputable
{
  public:
    MachineTypeComputable(RobotMemory* robot_memory, fawkes::Logger* logger, fawkes::Configuration* config);
    virtual ~MachineTypeComputable();

  private:
    std::list<mongo::BSONObj> compute_machine_type(mongo::BSONObj query, std::string collection);

    RobotMemory* robot_memory_;
    fawkes::Logger* logger_;
    const char* name_ = "PddlFromOrder: RM-MachineTypeComputable";
    std::vector<Computable*>  computables;
    fawkes::Configuration* config_;
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_MACHINE_TYPE_COMPUTABLE_H_ */
