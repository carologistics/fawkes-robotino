
/***************************************************************************
 *  pddl-from-order_thread.cpp - pddl-from-order
 *
 *  Created: Sat Mar 18 19:46:59 2017
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

#include "pddl-from-order_thread.h"

using namespace fawkes;
using namespace mongo;

/** @class PddlFromOrderThread 'pddl-from-order_thread.h' 
 * Generates a PDDL problem from an incoming RCLL order
 * @author Matthias Loebach
 */

/** Constructor. */
PddlFromOrderThread::PddlFromOrderThread()
 : Thread("PddlFromOrderThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("PddlPlannerThread")
{
}

void
PddlFromOrderThread::init()
{
  std::string cfg_prefix = "plugins/pddl-from-order/";
  cfg_wm_collection_ = config->get_string(cfg_prefix + "wm-collection").c_str();
  cfg_order_plan_threshold_ = config->get_uint(cfg_prefix + "order-plan-threshold");

  plan_if_ = blackboard->open_for_reading<PddlPlannerInterface>(config->get_string(cfg_prefix + "planner-interface").c_str());

  gen_if_ = blackboard->open_for_reading<PddlGenInterface>(
      config->get_string(cfg_prefix + "generator-interface").c_str());
  bbil_add_data_interface(gen_if_);

  order_trigger_ = robot_memory->register_trigger(
      fromjson("{relation:\"order\"}"), cfg_wm_collection_,
      &PddlFromOrderThread::retrieve_new_order, this);
}

void
PddlFromOrderThread::loop()
{
  if ( pddl_gen_running_ ) {
    logger->log_error(name(), "PPDL generation running, skipping loop for now.");
    wait_for_wakeup_ = true;
    return;
  } else {
    logger->log_error(name(), "No PDDL generation running, starting loop");
  }
  logger->log_info(name(),"PDDL generation finished, starting planning.");

  plan_if_->msgq_enqueue(new PddlPlannerInterface::PlanMessage());
}

void
PddlFromOrderThread::finalize()
{
}

void
PddlFromOrderThread::retrieve_new_order(BSONObj doc)
{
  gen_if_->msgq_enqueue(new PddlGenInterface::GenerateMessage());

  orders_recv_++;
  logger->log_info(name(), "Received order %d", orders_recv_);

  wait_for_wakeup_ = true;
}

void
PddlFromOrderThread::try_wakeup() {
  if ( pddl_gen_running_ ) {
    logger->log_error(name(), "PDDL Generation running, cannot re-generate now.");
    return;
  }
  if ( wait_for_wakeup_
      && orders_recv_ >= cfg_order_plan_threshold_ ) {
    logger->log_info(name(), "Calling plan generator with new order");
    wakeup();
    wait_for_wakeup_ = false;
  }
}

void
PddlFromOrderThread::bb_interface_data_changed(Interface *interface) throw()
{
  if ( interface->uid() == gen_if_->uid() ) {
    gen_if_->read();
    if ( gen_if_->is_final() ) {
      pddl_gen_running_ = false;
      try_wakeup();
      logger->log_info(name(), "PDDL generation is finished, waking up loop.");
    } else {
      pddl_gen_running_ = true;
      logger->log_info(name(), "PDDL generation started.");
    }
  } else {
    logger->log_error(name(), "Data changed for unknown interface");
  }
}
