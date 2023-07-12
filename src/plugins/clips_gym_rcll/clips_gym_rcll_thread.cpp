/***************************************************************************
 *  clips_gym_rcll_thread.cpp -
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

#include "clips_gym_rcll_thread.h"

#include <iostream>
#include <regex>
#include <string>

//for calling boost python from plugin
//#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <core/threading/mutex_locker.h>
#include <pybind11/embed.h>

#include <boost/bind/bind.hpp>
#include <clipsmm.h>
//#include "goalAction.h"

//#include "clips-observation-info.h"

using namespace std;
//using namespace boost::python;
namespace py = pybind11;

using namespace fawkes;

/** @class ClipsGymRCLLThread
 *  The plugin thread, initializes the aspect.
 *
 *  @author
 */

constexpr char ClipsGymRCLLThread::cfg_prefix_[];

ClipsGymRCLLThread::ClipsGymRCLLThread() : ClipsGymThread("ClipsGymRCLLThread", "CLipsGymRCLL") // TODO: put as aspect instead?
{
}

ClipsGymRCLLThread *ClipsGymRCLLThread::rcll_thread_instance{nullptr};
std::mutex          ClipsGymRCLLThread::mutex;

ClipsGymRCLLThread *
ClipsGymRCLLThread::getInstance()
{
	std::cout << "ClipsGymRCLLThread: getInstance start" << std::endl;
	std::lock_guard<std::mutex> lock(mutex);
	if (rcll_thread_instance == nullptr) {
		rcll_thread_instance = new ClipsGymRCLLThread();
	}
	return rcll_thread_instance;
}

PYBIND11_MODULE(clips_gym_rcll, m)
{
	m.doc() = "ClipsGymRCLLThread"; // optional module docstring
	py::class_<ClipsGymRCLLThread>(m, "ClipsGymThread")
	  .def(py::init<>())
	  .def("getInstance", &ClipsGymRCLLThread::getInstance, py::return_value_policy::reference)
	  .def("step", &ClipsGymRCLLThread::step)
	  .def("generateActionSpace", &ClipsGymRCLLThread::generateActionSpace)
	  .def("getRefboxGameTime", &ClipsGymRCLLThread::getRefboxGameTime)
	  .def("getRefboxGamePhase", &ClipsGymRCLLThread::getRefboxGamePhase)
	  .def("clipsGymRCLLSleep", &ClipsGymRCLLThread::clipsGymRCLLSleep)
	  .def("log", &ClipsGymRCLLThread::log);
}

void
ClipsGymRCLLThread::init()
{
}

/* Checks if rl-waiting fact exists and sleeps */
void
ClipsGymRCLLThread::loop()
{
	std::cout << "In Loop ClipsGymRCLLThread" << std::endl;
}

void
ClipsGymRCLLThread::finalize()
{
	//clips.lock();
	//clips->assert_fact("(executive-finalize)");
	//clips.unlock();
}

// TODO: do we also need to sleep this thread?
void
ClipsGymRCLLThread::clipsGymRCLLSleep(int milliseconds)
{
	logger->log_info(name(), "ClipsGym Sleeping for %i ms", milliseconds);
	std::this_thread::sleep_for(milliseconds * 1ms);
}

//std::string
ClipsObservationInfo
ClipsGymRCLLThread::step(std::string next_goal)
{
	std::cout << "In ClipsGymRCLLThread step function" << std::endl;
	std::cout << "next_goal from python: " << next_goal << std::endl;
	ClipsObservationInfo obs_info = ClipsObservationInfo();
	obs_info.reward               = 0;
	//Transform string to goal
	//std::string n_goal = "TOWER-C1#b#d#";
	//std::string goalID = getGoalId(next_goal);

	std::string goalID = getGoalIdByString(currentExecutableGoals, next_goal);

	if (goalID == "") {
		std::cout << "Goal id not found!" << std::endl;
		std::string env_state = create_rl_env_state_from_facts();
		std::cout << "End Clips Gym Thread step function" << std::endl;

		obs_info.observation = env_state;
		return obs_info;
	}
	assertRlGoalSelectionFact(goalID);
	std::cout << "fact asserted, start running clips" << std::endl;

	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	//Check frequently if the selected goal is evaluated
	bool  env_feedback = false;
	float speed        = config->get_float("/env/speed");
	float max_time     = config->get_float("/env/step/max_time");  //60 sec without speedup
	float wait_time    = config->get_float("/env/step/wait_time"); //5 sec
	if (speed != 0.0 || speed != 1.0) {
		wait_time = wait_time / speed;
	}
	int  elapsed_time        = 0;
	bool check_for_game_over = false;
	while (!env_feedback && elapsed_time < max_time) {
		std::this_thread::sleep_for(wait_time * 1000ms);
		clips.lock();
		clips->evaluate("(printout t \"In Sleeping Step Function \" crlf) ");
		CLIPS::Fact::pointer fact = clips->get_facts();

		while (fact) {
			CLIPS::Template::pointer tmpl    = fact->get_template();
			std::size_t              found   = tmpl->name().find("rl-finished-goal");
			std::size_t              wm_fact = tmpl->name().find("wm-fact");
			if (found != std::string::npos) {
				std::string goalID  = getClipsSlotValuesAsString(fact->slot_value("goal-id"));
				std::string outcome = getClipsSlotValuesAsString(fact->slot_value("outcome"));
				std::string result  = getClipsSlotValuesAsString(fact->slot_value("result"));
				std::cout << "In ClipsGymRCLLThread step: Goal: " << goalID
				          << " is evaluated with outcome: " << outcome << " result: " << result
				          << std::endl;

				obs_info.reward = fact->slot_value("result")[0].as_integer();
				//TODO check why its no valid syntax
				obs_info.info = "Goal-id " + goalID + " Outcome " + outcome; //TODO add also goal-id to info
				//TODO: check if goalID is the same
				env_feedback = true;
				fact->retract();
				std::cout << "In ClipsGymRCLLThread step: after retracting rl-finished-goal fact"
				          << std::endl;
				break;
			} else if (check_for_game_over && wm_fact != std::string::npos) {
				//(wm-fact (id "/refbox/phase") (key refbox phase) (type UNKNOWN) (is-list FALSE) (value POST_GAME) (values))
				std::string key   = getClipsSlotValuesAsString(fact->slot_value("key"));
				std::string value = getClipsSlotValuesAsString(fact->slot_value("value"));
				//key: refbox#phase id:/refbox/phase value: PRODUCTION
				if (key == "refbox#phase" && (value == "POST_GAME" || value == "SETUP")) {
					obs_info.info = "Game Over";
					logger->log_info(name(), "Step Function: %s %s Game Over", key.c_str(), value.c_str());
					env_feedback = true;
					break;
				}
			}
			fact = fact->next();
		}

		//TODO: check outcome - set return 1 for completed and 0 otherwise

		clips.unlock();
		elapsed_time += wait_time;
		check_for_game_over = true;
	}
	if (!env_feedback) {
		logger->log_error(name(), "Ending step function without finished goal!!!");
	}
	std::string env_state = create_rl_env_state_from_facts();

	std::cout << "End Clips Gym Thread step function" << std::endl;

	obs_info.observation = env_state;
	//ClipsObservationInfo obs_info = ClipsObservationInfo(env_state);
	return obs_info; //env_state;
}

py::list
ClipsGymRCLLThread::generateActionSpace()
{
	std::cout << "Clips Gym Thread generateActionSpace start" << std::endl;
	//TODO temporär drin
	std::vector<std::string> formulated_goals = getAllFormulatedGoals();
	for (std::string s : formulated_goals) {
		std::cout << s << std::endl;
	}

	//TODO: implement generation based on clips goals
	/*std::string space[] = {"TOWER-C1#buttom#a#top#c",
	                       "TOWER-C1#buttom#b#top#d",
	                       "TOWER-C1#buttom#e#top#d",
	                       "TOWER-C2#buttom#b#middle#d#top#e"}; //, "TOWER-C1#buttom#a#top#e"};
*/
	std::string space[] = {"ENTER-FIELD",
	                       "BUFFER-CAP#cap-color#CAP_BLACK",
	                       "BUFFER-CAP#cap-color#CAP_GREY",
	                       "MOUNT-CAP#wp-loc#C-BS",
	                       "MOUNT-CAP#wp-loc#C-CS1",
	                       "MOUNT-CAP#wp-loc#C-CS2",
	                       "MOUNT-CAP#wp-loc#C-RS1",
	                       "MOUNT-CAP#wp-loc#C-RS2",
	                       "MOUNT-CAP#wp-loc#C-SS",
	                       "DISCARD#wp-loc#C-BS",
	                       "DISCARD#wp-loc#C-CS1",
	                       "DISCARD#wp-loc#C-CS2",
	                       "DISCARD#wp-loc#C-DS",
	                       "DISCARD#wp-loc#C-RS1",
	                       "DISCARD#wp-loc#C-RS2",
	                       "DISCARD#wp-loc#C-SS",
	                       "PAY-FOR-RINGS-WITH-BASE#target-mps#C-RS1",
	                       "PAY-FOR-RINGS-WITH-BASE#target-mps#C-RS2",
	                       "PAY-FOR-RINGS-WITH-CAP-CARRIER#target-mps#C-RS1",
	                       "PAY-FOR-RINGS-WITH-CAP-CARRIER#target-mps#C-RS2",
	                       "PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF#target-mps#C-RS1",
	                       "PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF#target-mps#C-RS2",
	                       "MOUNT-RING#ring-color#RING_BLUE#wp-loc#C-BS",
	                       "MOUNT-RING#ring-color#RING_BLUE#wp-loc#C-RS1",
	                       "MOUNT-RING#ring-color#RING_BLUE#wp-loc#C-RS2",
	                       "MOUNT-RING#ring-color#RING_GREEN#wp-loc#C-BS",
	                       "MOUNT-RING#ring-color#RING_GREEN#wp-loc#C-RS1",
	                       "MOUNT-RING#ring-color#RING_GREEN#wp-loc#C-RS2",
	                       "MOUNT-RING#ring-color#RING_ORANGE#wp-loc#C-BS",
	                       "MOUNT-RING#ring-color#RING_ORANGE#wp-loc#C-RS1",
	                       "MOUNT-RING#ring-color#RING_ORANGE#wp-loc#C-RS2",
	                       "MOUNT-RING#ring-color#RING_YELLOW#wp-loc#C-BS",
	                       "MOUNT-RING#ring-color#RING_YELLOW#wp-loc#C-RS1",
	                       "MOUNT-RING#ring-color#RING_YELLOW#wp-loc#C-RS2",
	                       "DELIVER",
	                       "WAIT-NOTHING-EXECUTABLE",
	                       "MOVE-OUT-OF-WAY"};

	py::list action_space;
	for (std::string s : space) {
		action_space.append((py::str)s);
	}
	return action_space;
}

//py::list
//ClipsGymRCLLThread::generateObservationSpace()
//{
//	//TODO: implement generation based on clips facts
//	std::string space[] = {"clear(a)",   "clear(b)",         "clear(c)",        "clear(d)",
//	                       "clear(e)",   "handempty(robo1)", "handfull(robo1)", "holding(a)",
//	                       "holding(b)", "holding(c)",       "holding(d)",      "holding(e)",
//	                       "on(a,b)",    "on(a,c)",          "on(a,d)",         "on(a,e)",
//	                       "on(b,a)",    "on(b,c)",          "on(b,d)",         "on(b,e)",
//	                       "on(c,a)",    "on(c,b)",          "on(c,d)",         "on(c,e)",
//	                       "on(d,a)",    "on(d,b)",          "on(d,c)",         "on(d,e)",
//	                       "on(e,a)",    "on(e,b)",          "on(e,c)",         "on(e,d)",
//	                       "ontable(a)", "ontable(b)",       "ontable(c)",      "ontable(d)",
//	                       "ontable(e)"};
//	py::list    obs_space;
//	for (std::string s : space) {
//		obs_space.append((py::str)s);
//	}
//	return obs_space;
//}

void
ClipsGymRCLLThread::log(std::string log_msg)
{
	if (log_msg != "") {
		logger->log_info(name(), "RL: %s", log_msg.c_str());
	}
}

int
ClipsGymRCLLThread::getRefboxGameTime()
{
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer fact = clips->get_facts();
	int                  sec  = 0;
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		//(wm-fact (id "/refbox/game-time") (key refbox game-time) (type UINT) (is-list TRUE) (value nil) (values 68 239942.0))
		std::size_t found = tmpl->name().find("wm-fact");

		if (found != std::string::npos) {
			std::string key = getClipsSlotValuesAsString(fact->slot_value("key"));
			if (key == "refbox#game-time") {
				logger->log_info(name(), "get refbox game-time key: %s", key.c_str());
				std::vector<CLIPS::Value> slot_values = fact->slot_value("values");
				//Normal game 20min = 1200 sec - extension by 5 more min -  25min = 1500sec

				if (slot_values.size() > 0) {
					auto v = slot_values.at(0);
					logger->log_info(name(), "value type %d", v.type());
					switch (v.type()) {
					case CLIPS::TYPE_FLOAT: {
						std::cout << "v is float" << std::endl;
						float clips_time = v.as_float();
						sec              = static_cast<int>(clips_time);
					} break;
					case CLIPS::TYPE_INTEGER: {
						std::cout << "v is int" << std::endl;
						sec = int(v.as_integer());
					} break;
						/*case CLIPS::TYPE_SYMBOL:
					case CLIPS::TYPE_STRING:
					case CLIPS::TYPE_EXTERNAL_ADDRESS:
					case CLIPS::TYPE_INSTANCE_ADDRESS:
					case CLIPS::TYPE_INSTANCE_NAME:*/
					default:
						//sec = std::stoi(v.as_string());
						sec = 4;
						std::cout << "Clips value " << v.as_string() << std::endl;
					}
				}
				logger->log_info(name(), "Refbox game time %i sec", sec);
				break;
			}
		}

		fact = fact->next();
	}
	clips.unlock();
	return sec;
}

std::string
ClipsGymRCLLThread::getRefboxGamePhase()
{
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer fact  = clips->get_facts();
	std::string          phase = "None";
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		//(wm-fact (id "/refbox/game-time") (key refbox game-time) (type UINT) (is-list TRUE) (value nil) (values 68 239942.0))
		std::size_t found = tmpl->name().find("wm-fact");

		if (found != std::string::npos) {
			std::string key = getClipsSlotValuesAsString(fact->slot_value("key"));
			if (key == "refbox#phase") {
				phase = getClipsSlotValuesAsString(fact->slot_value("value"));
				logger->log_info(name(), "get refbox phase %s", phase.c_str());
				break;
			}
		}

		fact = fact->next();
	}
	clips.unlock();
	return phase;
}
