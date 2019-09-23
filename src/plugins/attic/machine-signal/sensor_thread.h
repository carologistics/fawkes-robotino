
/***************************************************************************
 *  sensor_thread.h - Laser thread that puses data into the interface
 *
 *  Created: Wed Oct 08 13:32:34 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *                  2014  Victor Mataré
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

#ifndef __PLUGINS_MACHINE_SIGNAL_SENSOR_THREAD_H_
#define __PLUGINS_MACHINE_SIGNAL_SENSOR_THREAD_H_

#include "pipeline_thread.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>

#include <string>

namespace fawkes {
class Position3DInterface;
}
namespace firevision {
class Camera;
}

class MachineSignalSensorThread : public fawkes::Thread,
                                  public fawkes::BlockedTimingAspect,
                                  public fawkes::LoggingAspect,
                                  public fawkes::ConfigurableAspect,
                                  public fawkes::BlackBoardAspect
{
public:
	MachineSignalSensorThread(MachineSignalPipelineThread *pipeline_thread);

	virtual void init();
	virtual void finalize();
	virtual void loop();

protected:
	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	virtual void
	run()
	{
		Thread::run();
	}

private:
	std::vector<fawkes::RobotinoLightInterface *> bb_signal_states_;
	fawkes::RobotinoLightInterface *              bb_signal_compat_;
	MachineSignalPipelineThread *                 pipeline_thread_;
};

#endif /* __PLUGINS_MACHINE_SIGNAL_SENSOR_THREAD_H_ */
