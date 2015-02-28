
/***************************************************************************
 *  sensor_thread.cpp - Pan/tilt plugin sensor thread
 *
 *  Created: Sat Feb 28 10:24:26 2015
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

#include "sensor_thread.h"
#include "act_thread.h"

using namespace fawkes;

/** @class GripperSensorThread "sensor_thread.h"
 * Katana sensor thread.
 * This thread integrates into the Fawkes main loop at the SENSOR hook and
 * triggers the act threads to retrieve and write new sensor data.
 * @author Tim Niemueller, Nicolas Limpert
 */

/** Constructor. */
GripperSensorThread::GripperSensorThread()
  : Thread("GripperSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
}


/** Add an act thread.
 * @param act_thread to add
 */
void
GripperSensorThread::add_act_thread(GripperAX12AThread *act_thread)
{
  __act_threads.push_back(act_thread);
}


void
GripperSensorThread::loop()
{
  for (__ati = __act_threads.begin(); __ati != __act_threads.end(); ++__ati) {
    try {
      (*__ati)->update_sensor_values();
    } catch (Exception &e) {
      logger->log_warn(name(), "Act thread %s threw an exception when updating sensor values",
		       (*__ati)->name());
      logger->log_warn(name(), e);
    }
  }
}
