/***************************************************************************
 *  sensor_thread.cpp - Laser thread that puses data into the interface
 *
 *  Created: Wed Oct 08 13:32:57 2008
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
 *             2012-2013  Johannes Rothe
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

#include <interfaces/Position3DInterface.h>

#include <cmath>
#include <cstdio>
#include <stdlib.h>
#include <string>

using namespace fawkes;
using namespace std;
/** @class OmniVisionPucksSensorThread "sensor_thread.h"
 * Laser sensor thread.
 * This thread integrates into the Fawkes main loop at the sensor hook and
 * publishes new data when available from the LaserAcquisitionThread.
 * @author Tim Niemueller
 * @author Johannes Rothe
 */

/** Constructor.
 * @param cfg_name short name of configuration group
 * @param cfg_prefix configuration path prefix
 * @param aqt LaserAcquisitionThread to get data from
 */
OmniVisionPucksSensorThread::OmniVisionPucksSensorThread(OmniVisionPucksPipelineThread *aqt)
: Thread("OmniVisionPucksSensorThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
	__aqt      = aqt;
	cfg_frame_ = ("/hardware/robotino/omnivision/frame");
}

void
OmniVisionPucksSensorThread::init()
{
}

void
OmniVisionPucksSensorThread::finalize()
{
}

void
OmniVisionPucksSensorThread::loop()
{
	if (__aqt->lock_if_new_data()) {
		// Iterator over the List in the Pipeline Thread whilst iterating over the
		// sensor thread list
		std::list<fawkes::Position3DInterface *>::iterator pipeline_pucks;
		for (pipeline_pucks = __aqt->puck_ifs_.begin(); pipeline_pucks != __aqt->puck_ifs_.end();
		     pipeline_pucks++) {
			(*pipeline_pucks)->write();

			// experimental comment, I am not sure of the consequences
			// (I am not sure if we need this here at all..) --DE
			//(*pipeline_pucks)->set_translation(0,0);
			//(*pipeline_pucks)->set_translation(1,0);
		}
	}
	__aqt->unlock();
}
