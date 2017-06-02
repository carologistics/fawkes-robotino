/***************************************************************************
 *  order_computable.cpp - Computable for retrieving a full order
 *    
 *
 *  Created: Tue 30 May 2017 22:44:47 CEST
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

#include "order_computable.h"

using namespace fawkes;
using namespace mongo;

/** @class OrderComputable  order_computable.h
 * Computable aggregating  a full order including all product properties
 * @author Matthias Loebach
 */

OrderComputable::OrderComputable(RobotMemory* robot_memory, fawkes::Logger* logger, fawkes::Configuration* config)
{
  robot_memory_ = robot_memory;
  logger_ = logger;
  config_ = config;

  //register computable
  Query query = fromjson("{relation:\"aggregated_order\"}");
  std::vector<std::string>  collections = config->get_strings("plugins/robot-memory/computables/order/collections");
  int priority = config->get_int("plugins/robot-memory/computables/order/priority");
  float caching_time = config->get_float("plugins/robot-memory/computables/order/caching-time");
  for( std::string col : collections )
  {
    computables.push_back(robot_memory_->register_computable(query, col, &OrderComputable::compute_order, this, caching_time, priority));
  }

  logger_->log_info(name_, "Registered OrderComputable");
}

OrderComputable::~OrderComputable()
{
  for( Computable *comp : computables )
  {
    robot_memory_->remove_computable(comp);
  }
}

std::list<BSONObj> OrderComputable::compute_order(BSONObj query, std::string collection)
{
  Query clean_query = fromjson("{relation:\"order\"}");
  QResCursor cur = robot_memory_->query(clean_query, collection);

  std::list<BSONObj> res;
  while( cur->more() ) {
    BSONObj order = cur->next();
    BSONElement id_elem = order.getField("product-id");
    long long product_id = id_elem.numberLong();
    BSONObj product_query = fromjson("{relation:\"product\", id: " + std::to_string(product_id)
        + ", \"product-id\": 0}");
    QResCursor prod_cur = robot_memory_->query(product_query, collection);
    BSONObj product = prod_cur->next();
    BSONObjBuilder full_order;
    //full_order.append(order.getField("relation"));
    full_order << "relation" << "aggregated_order";
    full_order.append(order.getField("id"));
    full_order.append(order.getField("complexity"));
    std::vector<BSONElement> rings = product.getField("rings").Array();
    BSONArrayBuilder new_rings;
    for ( auto& r : rings ) {
      new_rings.append(r);
    }
    for ( int i = 3 - rings.size(); i > 0 ; i-- ) {
      new_rings.append("NONE");
    }
    full_order.append("rings", new_rings.arr());
    full_order.append(product.getField("cap"));
    full_order.append(product.getField("base"));
    full_order << "aggregated" << true;

    res.push_back(full_order.obj());
  }
  return res;
}
