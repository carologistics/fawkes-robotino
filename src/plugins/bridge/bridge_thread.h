/***************************************************************************
 *  bridge_thread.h - Thread to print the robot's position to the log
 *
*  Created: Thu Aug  2015
 *  Copyright  2015 Mostafa Gomaa
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

#ifndef __PLUGINS_BRIDGE_BRIDGE_THREAD_H_
#define __PLUGINS_BRIDGE_BRIDGE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/clips/aspect/clips_manager.h>


#include <vector>
#include <string>

namespace fawkes {
  class Position3DInterface;
}

class Dispatcher;

class BridgeThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::CLIPSManagerAspect
{
 public:
  BridgeThread();
  virtual ~BridgeThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
 private:

  Dispatcher   *dispatcher_;

};

#endif
