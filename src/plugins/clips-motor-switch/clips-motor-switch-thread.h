
/***************************************************************************
 *  clips-motor-switch-thread.h - Switch motor from CLIPS
 *
 *  Created: Thu Apr 25 12:30:30 2013
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_MOTOR_SWITCH_CLIPS_MOTOR_SWITCH_THREAD_H_
#define __PLUGINS_CLIPS_MOTOR_SWITCH_CLIPS_MOTOR_SWITCH_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <string>
#include <vector>

namespace fawkes {
class MotorInterface;
}

class ClipsMotorSwitchThread : public fawkes::Thread,
                               public fawkes::LoggingAspect,
                               public fawkes::ConfigurableAspect,
                               public fawkes::BlackBoardAspect,
                               public fawkes::CLIPSFeature,
                               public fawkes::CLIPSFeatureAspect
{
public:
	ClipsMotorSwitchThread();
	virtual ~ClipsMotorSwitchThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	// for CLIPSFeature
	virtual void clips_context_init(const std::string &                  env_name,
	                                fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void clips_motor_enable(std::string env_name);
	void clips_motor_disable(std::string env_name);

	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> envs_;

private:
	std::string             cfg_iface_id_;
	fawkes::MotorInterface *motor_if_;
};

#endif
