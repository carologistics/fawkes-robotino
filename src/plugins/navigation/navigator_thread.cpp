
/***************************************************************************
 *  navigator_thread.cpp - Robotino Navigator Thread
 *
 *  Created: Mon May 14 16:53:27 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "navigator_thread.h"

#include <interfaces/NavigatorInterface.h>
#include <interfaces/MotorInterface.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/RobotinoSensorInterface.h>

using namespace fawkes;

/** @class NavigatorThread "navigator_thread.h"
 * Navigator thread.
 * @author Tim Niemueller
 */

/** Contructor. */
NavigatorThread::NavigatorThread()
  : Thread("NavigatorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

/** Destructor. */
NavigatorThread::~NavigatorThread()
{
}


void
NavigatorThread::init()
{
  // navigator interface (writer)
  try {
    nav_if_ = blackboard->open_for_writing<NavigatorInterface>("Navigator");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open navigator "
             "interface for writing", name());
    logger->log_error(name(), e);
    throw;
  }

  // motor interface (reader)
  try {
    mot_if_ = blackboard->open_for_reading<MotorInterface>("Motor");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open motor "
             "interface for reading", name());
    logger->log_error( name(), e );
    throw;
  }

  // object position interfaces (obstacles)
  try {
    obs_ifs_ = blackboard->open_multiple_for_reading<Position3DInterface>("OmniObstacle*");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open object "
             "interface for reading", name());
    logger->log_error( name(), e);
    throw;
  }

  // object position interface (puck)
  try {
    puckpos_if_ = blackboard->open_for_reading<Position3DInterface>("Puck");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open object position "
             "interface for reading", name());
    logger->log_error(name(), e);
    throw;
  }

  // setup interface observer
  bbio_add_observed_create("Position3DInterface", "OmniObstacle *" );
  blackboard->register_observer(this);
}


void
NavigatorThread::finalize()
{
  blackboard->unregister_observer(this);

  // close interfaces
  try {
    blackboard->close(nav_if_);
    blackboard->close(mot_if_);
    std::list<fawkes::Position3DInterface *>::iterator oili;
    for ( oili = obs_ifs_.begin(); oili != obs_ifs_.end(); ++oili) {
      blackboard->close(*oili);
    }
    blackboard->close(puckpos_if_);
  } catch (Exception& e) {
    logger->log_error( name(), "Closing interface failed!" );
    logger->log_error( name(), e );
  }
}

void
NavigatorThread::bb_interface_created(const char *type, const char *id) throw()
{
  try {
    obs_ifs_.push_back(blackboard->open_for_reading<Position3DInterface>(id));

    logger->log_info(name(), "Opened interface %s::%s", type, id);

  } catch (Exception& e) {
    logger->log_warn(name(), "Failed to open interface %s::%s", type, id);
  }
}


void
NavigatorThread::loop()
{

  // update all reading interfaces
  mot_if_->read();
  puckpos_if_->read();
  sens_if_->read();
  std::list<fawkes::Position3DInterface *>::iterator o;
  for (o = obs_ifs_.begin(); o != obs_ifs_.end(); ++o) {
    (*o)->read();
  }


  // process incoming messages
  while ( ! nav_if_->msgq_empty()) {

    // stop
    if (NavigatorInterface::StopMessage *msg = nav_if_->msgq_first_safe(msg)) {
      logger->log_info( name(), "Stop message received" );
    }

    // cartesian goto
    else if (NavigatorInterface::CartesianGotoMessage *msg = nav_if_->msgq_first_safe(msg)) {
      logger->log_info( name(), 
                        "Cartesian goto message received (x,y) = (%f,%f)",
                        msg->x(), msg->y() );
	  
    }

    // polar goto
    else if (NavigatorInterface::PolarGotoMessage *msg = nav_if_->msgq_first_safe(msg)) {
      logger->log_info( name(),
                        "Polar goto message received (phi,dist) = (%f,%f)",
                        msg->phi(), msg->dist() );
    }
	  
    // max velocity
    else if (NavigatorInterface::SetMaxVelocityMessage *msg = nav_if_->msgq_first_safe(msg)) {
      logger->log_info( name(),
                        "velocity message received %f",
                        msg->max_velocity() );
    }
      
    nav_if_->msgq_pop();
  }

  /* Processing code goes here.
   * This should also include setting appropriate information on the
   * navigator interface (nav_if_).
   */

  // write current data to navigator interface
  nav_if_->write();
}
