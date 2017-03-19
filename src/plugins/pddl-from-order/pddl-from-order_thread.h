
/***************************************************************************
 *  pddl-from-order_thread.h - pddl-from-order
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

#ifndef __PLUGINS_PDDL_FROM_ORDER_THREAD_H_
#define __PLUGINS_PDDL_FROM_ORDER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>
#include <blackboard/interface_listener.h>
#include <interfaces/PddlPlannerInterface.h>
#include <interfaces/PddlGenInterface.h>


namespace fawkes {
  // add forward declarations here, e.g., interfaces
}

class PddlFromOrderThread 
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener,
  public fawkes::RobotMemoryAspect
{

 public:
  PddlFromOrderThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() { Thread::run(); }

 private:
  struct order {
    uint32_t id;
    uint8_t complexity;
    std::string base;
    std::vector<std::string> rings;
    std::string cap;
    uint8_t delivery_gate;
    uint32_t quantity;
    uint32_t begin;
    uint32_t end;
  };

  std::string cfg_wm_collection_;
  uint8_t cfg_order_plan_threshold_;

  fawkes::PddlPlannerInterface* plan_if_;
  fawkes::PddlGenInterface* gen_if_;
  EventTrigger* order_trigger_;
  
  bool pddl_gen_running_ = false;
  bool wait_for_wakeup_ = false;
  void retrieve_new_order(mongo::BSONObj doc);
  uint8_t orders_recv_ = 0;
  void try_wakeup();
  void bb_interface_data_changed(fawkes::Interface *interface) throw();
};


#endif
