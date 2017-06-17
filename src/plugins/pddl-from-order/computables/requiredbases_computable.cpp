/***************************************************************************
 *  requiredbases_computable.cpp - Computable for getting require bases per ring color
 *    
 *
 *  Created: Sat 17 Jun 2017 15:22:44 CEST
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

#include "requiredbases_computable.h"

using namespace fawkes;
using namespace mongo;

/** @class RequiredBasesComputable  reqbases_type_computable.h
 * Computable calculating symbolic representations for number of bases required
 * @author Matthias Loebach
 */

RequiredBasesComputable::RequiredBasesComputable(RobotMemory* robot_memory, fawkes::Logger* logger, fawkes::Configuration* config)
{
  robot_memory_ = robot_memory;
  logger_ = logger;
  config_ = config;

  //register computable
  Query query = fromjson("{relation:\"requiredbases\"}");
  std::vector<std::string>  collections = config->get_strings("plugins/robot-memory/computables/requiredbases/collections");
  int priority = config->get_int("plugins/robot-memory/computables/requiredbases/priority");
  float caching_time = config->get_float("plugins/robot-memory/computables/requiredbases/caching-time");
  for( std::string col : collections )
  {
    computables.push_back(robot_memory_->register_computable(query, col, &RequiredBasesComputable::compute_requiredbases, this, caching_time, priority));
  }

  logger_->log_info(name_, "Registered RequiredBasesComputable");
}

RequiredBasesComputable::~RequiredBasesComputable()
{
  for( Computable *comp : computables )
  {
    robot_memory_->remove_computable(comp);
  }
}

std::list<BSONObj> RequiredBasesComputable::compute_requiredbases(BSONObj query, std::string collection)
{
  BSONObjBuilder clean_query;
  clean_query << "relation" << "ring";
  QResCursor cur = robot_memory_->query(clean_query.obj(), collection);

  std::list<BSONObj> res;
  while( cur->more() ) {
    BSONObj reqbases = cur->next();
    BSONObjBuilder reqbases_obj;
    reqbases_obj << "relation" << "requiredbases";
    
    std::string color = "RING_";
    color += reqbases.getField("color").String();
    reqbases_obj << "color" << color;
    
    long base_num = reqbases.getField("req-bases").Long();
    std::string base_num_string;
    switch (base_num) {
      case 0: base_num_string = "ZERO"; break;
      case 1: base_num_string = "ONE"; break;
      case 2: base_num_string = "TWO"; break;
      case 3: base_num_string = "THREE"; break;
    }
    reqbases_obj << "num" << base_num_string;

    logger_->log_info(name_, "Computed ring %s with %s bases", color.c_str(), base_num_string.c_str());

    res.push_back(reqbases_obj.obj());
  }
  return res;
}
