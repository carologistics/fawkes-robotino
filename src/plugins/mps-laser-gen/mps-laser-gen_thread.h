
/***************************************************************************
 *  mps-laser-gen_thread.h - mps-laser-gen
 *
 *  Plugin created: Thu Jun 30 21:54:46 2016

 *  Copyright  2016  Tim Niemueller
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

#ifndef __PLUGINS_MPS_LASER_GEN_THREAD_H_
#define __PLUGINS_MPS_LASER_GEN_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <navgraph/aspect/navgraph.h>
#include <plugins/ros/aspect/ros.h>
#include <aspect/tf.h>

#include <string>

#include <interfaces/Laser360Interface.h>
#include <ros/publisher.h>

namespace fawkes {
}

class MPSLaserGenThread 
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
	public fawkes::BlackBoardAspect,
	public fawkes::NavGraphAspect,
	public fawkes::ROSAspect,
	public fawkes::TransformAspect
{

 public:
  MPSLaserGenThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() { Thread::run(); }

 private:
  fawkes::Laser360Interface* laser_if_;
  ros::Publisher vispub_;

};


#endif
