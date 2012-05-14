
/***************************************************************************
 *  navigator_thread.h - Robotino Navigator Thread
 *
 *  Created: Mon May 14 16:46:37 2012
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
 
#ifndef __NAVIGATOR_NAVIGATOR_THREAD_H_
#define __NAVIGATOR_NAVIGATOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <blackboard/interface_observer.h>

namespace fawkes {
  class NavigatorInterface;
  class MotorInterface;
  class Position3DInterface;
  class RobotinoSensorInterface;
}

class NavigatorThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardInterfaceObserver
{
 public:
  NavigatorThread();
  virtual ~NavigatorThread();

  /* thread */
  virtual void init();
  virtual void finalize();
  virtual void loop();
  
  /* interface observer */
  virtual void bb_interface_created(const char *type, const char *id) throw();

 private:
  fawkes::NavigatorInterface      *nav_if_;
  fawkes::MotorInterface          *mot_if_;
  fawkes::Position3DInterface     *puckpos_if_;
  fawkes::RobotinoSensorInterface *sens_if_;

  std::list<fawkes::Position3DInterface *> obs_ifs_;
};

#endif /* __NAVIGATOR_NAVIGATOR_THREAD_H_ */
