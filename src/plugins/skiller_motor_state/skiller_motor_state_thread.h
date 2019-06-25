
/***************************************************************************
 *  skiller_state_thread.h - Indicate skiller state through LED
 *
 *  Created: Fri Jun 14 15:04:53 2019
 *  Copyright  2019  Morian Sonnet
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

#ifndef __PLUGINS_SKILLER_MOTOR_STATE_THREAD_H_
#define __PLUGINS_SKILLER_MOTOR_STATE_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>

#include <interfaces/MotorInterface.h>
#include <interfaces/RobotinoSensorInterface.h>
#include <interfaces/SkillerInterface.h>

namespace fawkes {
class SkillerInterface;
class RobotinoSensorInterface;
} // namespace fawkes

class SkillerMotorStateThread : public fawkes::Thread,
                                public fawkes::LoggingAspect,
                                public fawkes::ConfigurableAspect,
                                public fawkes::BlockedTimingAspect,
                                public fawkes::BlackBoardAspect,
                                public fawkes::BlackBoardInterfaceListener {
public:
  SkillerMotorStateThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
  virtual void run() { Thread::run(); }

private:
  std::string cfg_skiller_ifid_;
  std::string cfg_rsens_ifid_;
  unsigned int cfg_digital_out_red_;
  unsigned int cfg_digital_out_yellow_;
  unsigned int cfg_digital_out_green_;
  fawkes::Time cfg_timeout_;

  fawkes::Time final_time_;
  fawkes::Time failed_time_;

  void enable(unsigned int output);
  void disable(unsigned int output);

  fawkes::SkillerInterface *skiller_if_;
  fawkes::RobotinoSensorInterface *rsens_if_;
};

#endif
