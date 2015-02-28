
/***************************************************************************
 *  act_thread.h - AX12 Gripper plugin act thread (abstract)
 *
 *  Created: Sat Feb 28 10:16:54 2015
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *                  2015  Nicolas Limpert
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

#ifndef __PLUGINS_GRIPPER_ACT_THREAD_H_
#define __PLUGINS_GRIPPER_ACT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

namespace fawkes {
  class GripperInterface;
}

class AX12GripperActThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  AX12GripperActThread(const char *thread_name);
  virtual ~AX12GripperActThread();

  //  virtual void update_sensor_values() = 0;

};


#endif
