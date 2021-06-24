
/***************************************************************************
 *  clips_tsp_thread.cpp -  TSP feature for CLIPS
 *
 *  Created: Wed Oct 09 19:27:41 2013
 *  Copyright  2021 Gjorgji Nikolovski
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

#include "clips_tsp_thread.h"
#include <clipsmm.h>

using namespace fawkes;

/** @class ClipsTSPThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Gjorgji Nikolovski
 */

/** Constructor. */
ClipsTSPThread::ClipsTSPThread()
: Thread("ClipsTSPThread", Thread::OPMODE_WAITFORWAKEUP),
  CLIPSFeature("tsp"),
  CLIPSFeatureAspect(this)
{
}

/** Destructor. */
ClipsTSPThread::~ClipsTSPThread()
{
}

void
ClipsTSPThread::init()
{
}

void
ClipsTSPThread::finalize()
{
	envs_.clear();
}

void
ClipsTSPThread::clips_context_init(const std::string &          env_name,
                                        LockPtr<CLIPS::Environment> &clips)
{
	envs_[env_name] = clips;
	logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());

	clips.lock();

	clips->add_function("solve_tsp",
	                    sigc::slot<std::string, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this, &ClipsTSPThread::solve_tsp),
	                      env_name)));

	clips.unlock();
}


std::string
ClipsTSPThread::solve_tsp(std::string env_name, std::string params) 
{
	char buffer[128];
	std::string result = "";
	std::string input = "python /home/robotino/fawkes-robotino/src/lua/skills/robotino/tsp_robotino.py" + params;
	// Open pipe to file
	FILE* pipe = popen(input.c_str(), "r");
	if (!pipe) {
		logger->log_error(name(),
	                 "Could not open python script");
		return "";
	}

	// read till end of process:
	while (!feof(pipe)) {

		// use buffer to read and add to result
		if (fgets(buffer, 128, pipe) != NULL)
			result += buffer;
	}

	pclose(pipe);
	logger->log_info(name(),
	                 "Env: %s, Result of solve_tsp call: %s",
	                 env_name.c_str(),
	                 result.c_str());
	return result;
}


void
ClipsTSPThread::clips_context_destroyed(const std::string &env_name)
{
	envs_.erase(env_name);
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
}



void
ClipsTSPThread::loop()
{
}
