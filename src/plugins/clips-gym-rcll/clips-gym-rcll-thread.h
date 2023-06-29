/***************************************************************************
 *  clips-gym-thread.h -
 *
 *  Created:
 *  Copyright
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

#include <aspect/aspect_provider.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <plugins/clips-gym/clips-gym-thread.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <chrono>
#include <map>
#include <mutex>
#include <thread>
//#include <plugins/clips/aspect/clips.h>

// for interaction with the CX
#include <clipsmm.h>

//#include <boost/python.hpp>
//namespace py = boost::python;
#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/embed.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

class ClipsGymRCLLThread : public ClipsGymThread
{
public:
	ClipsGymRCLLThread();

	void         init() override;
	virtual void finalize();
	virtual void loop();

	/*
	* Pybind 11 Module functions
	* - OpenAi Gym functions
	*/
	ClipsObservationInfo step(std::string next_goal);
	//std::string step(std::string next_goal);

	static ClipsGymRCLLThread *getInstance();
	py::list                   generateActionSpace();

	void log(std::string log_msg);

	int         getRefboxGameTime();
	std::string getRefboxGamePhase();

	void clipsGymRCLLSleep(int milliseconds);

private:
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> envs_;
	std::string                                                clips_env_name;
	constexpr static char cfg_prefix_[] = "/plugins/clips-gym/static/";

	//TODO extra Klasse auslagern
	std::map<std::string, std::vector<std::string>> paramTypeDomainObjectsMap;

	static ClipsGymRCLLThread *rcll_thread_instance;
	static std::mutex          mutex;

	std::vector<GoalAction> currentExecutableGoals;
};
//#endif
