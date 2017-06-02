/***************************************************************************
 *  machine_type_computable.cpp - Computable for retrieving a full machine_type
 *    
 *
 *  Created: Thu 01 Jun 2017 22:48:44 CEST
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

#include "machine_type_computable.h"

using namespace fawkes;
using namespace mongo;

/** @class MachineTypeComputable  machine_type_computable.h
 * Computable aggregating  a full machine_type including all product properties
 * @author Matthias Loebach
 */

MachineTypeComputable::MachineTypeComputable(RobotMemory* robot_memory, fawkes::Logger* logger, fawkes::Configuration* config)
{
  robot_memory_ = robot_memory;
  logger_ = logger;
  config_ = config;

  //register computable
  Query query = fromjson("{relation:\"machine_type\"}");
  std::vector<std::string>  collections = config->get_strings("plugins/robot-memory/computables/machine_type/collections");
  int priority = config->get_int("plugins/robot-memory/computables/machine_type/priority");
  float caching_time = config->get_float("plugins/robot-memory/computables/machine_type/caching-time");
  for( std::string col : collections )
  {
    computables.push_back(robot_memory_->register_computable(query, col, &MachineTypeComputable::compute_machine_type, this, caching_time, priority));
  }

  logger_->log_info(name_, "Registered MachineTypeComputable");
}

MachineTypeComputable::~MachineTypeComputable()
{
  for( Computable *comp : computables )
  {
    robot_memory_->remove_computable(comp);
  }
}

std::list<BSONObj> MachineTypeComputable::compute_machine_type(BSONObj query, std::string collection)
{
  BSONObjBuilder clean_query;
  clean_query << "relation" << "machine";
  clean_query.append(query.getField("team"));
  QResCursor cur = robot_memory_->query(clean_query.obj(), collection);

  std::list<BSONObj> res;
  while( cur->more() ) {
    BSONObj machine = cur->next();
    BSONObjBuilder machine_obj;
    machine_obj << "relation" << "machine_type";
    machine_obj.append(machine.getField("name"));
    machine_obj.append(machine.getField("team"));
    BSONElement type_elem = machine.getField("mtype");
    std::string type = type_elem.str();
    
    if ( type == "DS" ) {
      machine_obj << "mtype" << "delivery-station";
    } else if ( type == "CS" ) {
      machine_obj << "mtype" << "cap-station";
    } else if ( type == "RS" ) {
      machine_obj << "mtype" << "ring-station";
    } else if ( type == "BS" ) {
      machine_obj << "mtype" << "base-station";
    } else {
      machine_obj << "mtype" << "unknown-station";
    }
    res.push_back(machine_obj.obj());
  }
  return res;
}
