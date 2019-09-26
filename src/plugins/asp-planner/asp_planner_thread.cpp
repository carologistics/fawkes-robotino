
/***************************************************************************
 *  asp_planner_thread.cpp -  ASP-based planner main thread
 *
 *  Created on Thu Aug 18 04:20:02 2016
 *  Copyright (C) 2016 by Björn Schäpers
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

#include "asp_planner_thread.h"

#include "asp_planner_externals.h"

#include <core/threading/mutex_locker.h>

#include <algorithm>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/json.hpp>
#include <cstdlib>
#include <string>

using fawkes::MutexLocker;

/**
 * @brief Transforms the real time to ASP time steps.
 * @param[in] realGameTime The real game time in seconds.
 * @return The ASP time units.
 */
int
AspPlannerThread::realGameTimeToAspGameTime(const int realGameTime) const noexcept
{
	if (realGameTime == 0) {
		return 0;
	} // if ( realGameTime == 0 )
	return std::max(1,
	                realGameTime / TimeResolution
	                  + ((realGameTime % TimeResolution) * 2 >= TimeResolution ? 1 : 0));
}

/**
 * @brief Transforms the ASP time steps to real time.
 * @param[in] aspGameTime The ASP time units.
 * @return The real game time in seconds.
 */
int
AspPlannerThread::aspGameTimeToRealGameTime(const int aspGameTime) const noexcept
{
	return aspGameTime * TimeResolution;
}

/**
 * @brief Gets called, when a beacon is received, updates the robot information.
 * @param[in] document The DB document.
 */
void
AspPlannerThread::beaconCallback(const bsoncxx::document::view &document)
{
	try {
		const auto        object(document["o"]);
		const std::string name(object["name"].get_utf8().value.to_string());
		MutexLocker       locker(&WorldMutex);
		auto &            info = Robots[name];

		if (!info.Alive) {
			logger->log_info(LoggingComponent, "New robot %s detected.", name.c_str());
			setInterrupt(InterruptSolving::Critical, "New robot");
			info.AliveExternal = generateAliveExternal(name);
		} // if ( !info.Alive )
		info.LastSeen = Clock::now();
		info.Alive    = true;
		info.X        = static_cast<float>(object["x"].get_double());
		info.Y        = static_cast<float>(object["y"].get_double());
	} // try
	catch (const std::exception &e) {
		logger->log_error(LoggingComponent,
		                  "Exception while updating robot information: %s\n%s",
		                  e.what(),
		                  bsoncxx::to_json(document).c_str());
	} // catch ( const std::exception& e )
	catch (...) {
		logger->log_error(LoggingComponent,
		                  "Exception while updating robot information.\n%s",
		                  bsoncxx::to_json(document).c_str());
	} // catch ( ... )
	return;
}

/**
 * @brief Gets called, when a game state is received, updates the internal
 * game-time.
 * @param[in] document The DB document.
 */
void
AspPlannerThread::gameTimeCallback(const bsoncxx::document::view &document)
{
	try {
		const auto        object(document["o"]);
		const std::string phase(object["phase"].get_utf8().value.to_string());
		const int         gameTime(object["time"].get_int64());
		MutexLocker       locker(&WorldMutex);
		if (phase == "EXPLORATION") {
			GameTime = gameTime;
		} // if ( phase == "EXPLORATION" )
		else if (phase == "PRODUCTION") {
			GameTime = ExplorationTime + gameTime;
		} // else if ( phase == "PRODUCTION" )
		else if (phase == "POST_GAME") {
			GameTime = -1;
		} // else if ( phase == "POST_GAME" )
	}   // try
	catch (const std::exception &e) {
		logger->log_error(LoggingComponent,
		                  "Exception while updating game time: %s\n%s",
		                  e.what(),
		                  bsoncxx::to_json(document).c_str());
	} // catch ( const std::exception& e )
	catch (...) {
		logger->log_error(LoggingComponent,
		                  "Exception while updating game time.\n%s",
		                  bsoncxx::to_json(document).c_str());
	} // catch ( ... )
	return;
}

/**
 * @brief Gets called if there is a new information to a machine from the
 * refbox.
 * @param[in] document The information about the machine.
 */
void
AspPlannerThread::machineCallback(const bsoncxx::document::view &document)
{
	try {
		const auto object(document["o"]);
		// Remove the first 2 letters, because this is the team color and the dash.
		// We identify by "BS" not "C-BS".
		const std::string machine(object["machine"].get_utf8().value.to_string().substr(2));
		const std::string state(object["state"].get_utf8().value.to_string());
		MutexLocker       locker(&WorldMutex);
		auto &            info(Machines[machine]);
		if (state != info.State) {
			logger->log_warn(LoggingComponent,
			                 "Machine %s from %s to %s",
			                 machine.c_str(),
			                 info.State.c_str(),
			                 state.c_str());
			if (state == "BROKEN" || state == "DOWN") {
				static const int brokenTime = config->get_int("/asp-agent/working-durations/broken");
				if (info.WorkingUntil) {
					info.WorkingUntil -= GameTime;
				} // if ( info.WorkingUntil )
				info.BrokenUntil = GameTime + brokenTime;

				if (state == "BROKEN") {
					info.Prepared  = false;
					info.FillState = 0;
				} // if ( state == "BROKEN" )

				setInterrupt(InterruptSolving::Critical, "Machine broken/down");
			} // if ( state == "BROKEN" || state == "DOWN" )

			if (info.State == "BROKEN" || info.State == "DOWN") {
				if (info.WorkingUntil) {
					info.WorkingUntil += GameTime;
				} // if ( info.WorkingUntil )

				const auto diff(GameTime - info.BrokenUntil);
				logger->log_info(LoggingComponent,
				                 "Machine up again, now: %d, expected: %d, diff: %d",
				                 GameTime,
				                 info.BrokenUntil,
				                 diff);
				info.BrokenUntil = 0;

				if (diff <= -15) {
					setInterrupt(InterruptSolving::High, "Machine much earlier up again");
				} // if ( diff <= -15 )
				else if (diff <= -8) {
					setInterrupt(InterruptSolving::Normal, "Machine earlier up again");
				} // else if ( diff <= -8 )
			}   // if (info.State == "BROKEN" || info.State == "DOWN" )
		}     // if ( state != Machines[machine].State )
		info.State = std::move(state);
	} // try
	catch (const std::exception &e) {
		logger->log_error(LoggingComponent,
		                  "Exception while extracting machine info: %s\n%s",
		                  e.what(),
		                  bsoncxx::to_json(document).c_str());
	} // catch ( const std::exception& e )
	catch (...) {
		logger->log_error(LoggingComponent,
		                  "Exception while extracting machine info.\n%s",
		                  bsoncxx::to_json(document).c_str());
	} // catch ( ... )
	return;
}

/**
 * @brief Gets called if we got a new order.
 * @param[in] document The information about the order.
 */
void
AspPlannerThread::orderCallback(const bsoncxx::document::view &document)
{
	try {
		const auto                 object(document["o"]);
		const int                  number(object["number"].get_int64());
		const int                  quantity(object["quantity"].get_int64());
		const std::string          base(object["base"].get_utf8().value.to_string());
		const std::string          cap(object["cap"].get_utf8().value.to_string());
		const bsoncxx::array::view rings{object["rings"].get_array()};
		const int                  num_rings = std::distance(rings.begin(), rings.end());
		const std::string ring1(num_rings >= 1 ? rings[0].get_utf8().value.to_string() : "none");
		const std::string ring2(num_rings >= 2 ? rings[1].get_utf8().value.to_string() : "none");
		const std::string ring3(num_rings >= 3 ? rings[2].get_utf8().value.to_string() : "none");
		const int         delBegin(object["begin"].get_int64() + ExplorationTime);
		const int         delEnd(object["end"].get_int64() + ExplorationTime);

		MutexLocker locker(&WorldMutex);
		Orders.insert(
		  {number,
		   OrderInformation{
		     number, quantity, base, cap, std::string(), ring1, ring2, ring3, delBegin, delEnd}});

		if (RingColors.size() == 4) {
			// Do only spawn tasks when we have the ring infos.
			addOrderToASP(Orders[number]);
		} // if ( RingColors.size() > 4 )

		if (number > MaxOrders) {
			logger->log_error(LoggingComponent,
			                  "We expect no higher order numbers than %d, but got %d! This order "
			                  "will not be considered by the ASP program!",
			                  MaxOrders,
			                  number);
		} // if ( number > MaxOrders )
		if (quantity > MaxQuantity) {
			logger->log_error(LoggingComponent,
			                  "We expect no higher quantities for orders than %d, but got %d! This "
			                  "order will not be considered by the ASP program!",
			                  MaxQuantity,
			                  quantity);
		} // if ( quantity > MaxQuantity )
	}   // try
	catch (const std::exception &e) {
		logger->log_error(LoggingComponent,
		                  "Exception while extracting an order: %s\n%s",
		                  e.what(),
		                  bsoncxx::to_json(document).c_str());
	} // catch ( const std::exception& e )
	catch (...) {
		logger->log_error(LoggingComponent,
		                  "Exception while extracting an order.\n%s",
		                  bsoncxx::to_json(document).c_str());
	} // catch ( ... )
	return;
}

/**
 * @brief Gets called if we know all we need to know about a ring color.
 * @param[in] document The information about the color.
 */
void
AspPlannerThread::ringColorCallback(const bsoncxx::document::view &document)
{
	try {
		const auto        object(document["o"]);
		const std::string color(object["color"].get_utf8().value.to_string());
		const int         cost(object["cost"].get_int64());
		const std::string machine(object["machine"].get_utf8().value.to_string().substr(2));

		MutexLocker locker(&WorldMutex);
		RingColors.emplace_back(RingColorInformation{color, machine, cost});

		addRingColorToASP(RingColors.back());

		if (RingColors.size() == 4) {
			// We have the last ring info, spawn orders we have received until now.
			for (const auto &pair : Orders) {
				addOrderToASP(pair.second);
			} // for ( const auto& pair : Orders )
		}   // if ( RingColors.size() == 4 )
	}     // try
	catch (const std::exception &e) {
		logger->log_error(LoggingComponent,
		                  "Exception while setting ring color: %s\n%s",
		                  e.what(),
		                  bsoncxx::to_json(document).c_str());
	} // catch ( const std::exception& e )
	catch (...) {
		logger->log_error(LoggingComponent,
		                  "Exception while setting ring color.\n%s",
		                  bsoncxx::to_json(document).c_str());
	} // catch ( ... )
	return;
}

/**
 * @brief Gets called, when the team color is changed.
 * @param[in] document The document with the new color.
 */
void
AspPlannerThread::teamColorCallback(const bsoncxx::document::view &document)
{
	try {
		const auto           object(document["o"]);
		bsoncxx::array::view values{object["values"].get_array()};
		const std::string    color(values[0].get_utf8().value.to_string());
		if (TeamColor) {
			if (color == "nil") {
				TeamColor = nullptr;
				logger->log_info(LoggingComponent, "Unsetting Team-Color.");
			} // if ( color == "nil" )
			else {
				logger->log_info(LoggingComponent, "Changing Team-Color to %s.", color.c_str());
			} // else -> if ( color == "nil" )
			unsetTeam();
		} // if ( TeamColor )
		else {
			logger->log_info(LoggingComponent, "Setting Team-Color to %s.", color.c_str());
		} // else -> if ( TeamColor )

		if (color != "nil") {
			TeamColor = color == "CYAN" ? "C" : "M";
			setTeam();
		} // if ( color != "nil" )
	}   // try
	catch (const std::exception &e) {
		logger->log_error(LoggingComponent,
		                  "Exception while updating the team color: %s\n%s",
		                  e.what(),
		                  bsoncxx::to_json(document).c_str());
	} // catch ( const std::exception& e )
	catch (...) {
		logger->log_error(LoggingComponent,
		                  "Exception while updating the team color.\n%s",
		                  bsoncxx::to_json(document).c_str());
	} // catch ( ... )
	return;
}

/**
 * @brief Gets called, when the zones to explore are set.
 * @param[in] document The document with the zones.
 */
void
AspPlannerThread::zonesCallback(const bsoncxx::document::view &document)
{
	try {
		const auto                 object(document["o"]);
		const bsoncxx::array::view zonesArray{object["zones"].get_array()};
		std::vector<int>           zones;
		zones.reserve(std::distance(zonesArray.begin(), zonesArray.end()));
		std::transform(std::begin(zonesArray),
		               std::end(zonesArray),
		               std::back_inserter(zones),
		               [](const bsoncxx::array::element &zone) {
			               return std::stoi(zone.get_utf8().value.to_string().substr(1));
		               });
		const auto  begin = zones.begin();
		auto        end   = zones.end();
		MutexLocker locker(&WorldMutex);
		ReceivedZonesToExplore = true;
		for (auto zone = 1; zone <= 24; ++zone) {
			auto iter = std::find(begin, end, zone);
			if (iter == end) {
				releaseZone(zone, false);
			} // if ( iter == end )
			else {
				ZonesToExplore.push_back(zone);
				addZoneToExplore(zone);
				/* Move the found zone to the end, and move the end iterator one to the
         * left. So the std::find has to search a smaller range. */
				std::swap(*iter, *--end);
			} // else -> if ( iter == end )
		}   // for ( auto zone = 1; zone <= 24; ++zone )
		fillNavgraphNodesForASP(false);
	} // try
	catch (const std::exception &e) {
		logger->log_error(LoggingComponent,
		                  "Exception while extracting the zones to explore: %s\n%s",
		                  e.what(),
		                  bsoncxx::to_json(document).c_str());
	} // catch ( const std::exception& e )
	catch (...) {
		logger->log_error(LoggingComponent,
		                  "Exception while extracting the zones to explore.\n%s",
		                  bsoncxx::to_json(document).c_str());
	} // catch ( ... )
	return;
}

/**
 * @brief Constructor.
 */
AspPlannerThread::AspPlannerThread(void)
: Thread("AspPlannerThread", Thread::OPMODE_CONTINUOUS),
  ASPAspect("ASPPlanner", "ASP-Planner"),
  LoggingComponent("ASP-Planner-Thread"),
  ConfigPrefix("/asp-agent/"),
  TeamColor(nullptr),
  Unsat(0),
  // Config
  ExplorationTime(0),
  DeliverProductTaskDuration(0),
  FetchProductTaskDuration(0),
  LookAhaed(0),
  MaxDriveDuration(0),
  MaxOrders(0),
  MaxProducts(0),
  MaxQuantity(0),
  MaxTaskDuration(0),
  MaxWorkingDuration(0),
  PrepareCSTaskDuration(0),
  TimeResolution(0),
  // Worldmodel
  GameTime(0),
  ReceivedZonesToExplore(false),
  // Distances
  UpdateNavgraphDistances(true),
  // Requests
  Interrupt(InterruptSolving::Not),
  InterruptReason(""),
  SentCancel(false),
  // Solving, loop intern
  ProgramGrounded(false),
  ProductionStarted(false),
  // Solving
  NewSymbols(false),
  StartSolvingGameTime(0),
  // Plan
  PlanGameTime(0)
{
	return;
}

/**
 * @brief Destructor.
 */
AspPlannerThread::~AspPlannerThread(void)
{
	return;
}

/**
 * @brief Initliaizes the robmem callbacks and world model. Also calls the
 * specialized inits.
 */
void
AspPlannerThread::init(void)
{
	logger->log_info(LoggingComponent, "Initialize ASP Planner");
	loadConfig();
	Orders.reserve(MaxOrders);
	OrderTaskMap.reserve(MaxOrders * MaxQuantity);
	Products.reserve(MaxProducts);
	RingColors.reserve(4);
	Robots.reserve(PossibleRobots.size());
	ZonesToExplore.reserve(12);

	RobotMemoryCallbacks.reserve(8);

	using namespace bsoncxx::builder;
	RobotMemoryCallbacks.emplace_back(
	  robot_memory->register_trigger(basic::make_document(basic::kvp("relation", "active-robot"),
	                                                      basic::kvp("name",
	                                                                 [](basic::sub_document subdoc) {
		                                                                 subdoc.append(
		                                                                   basic::kvp("$ne", "RefBox"));
	                                                                 })),
	                                 "robmem.planner",
	                                 &AspPlannerThread::beaconCallback,
	                                 this));

	RobotMemoryCallbacks.emplace_back(
	  robot_memory->register_trigger(basic::make_document(basic::kvp("relation", "game-time")),
	                                 "robmem.planner",
	                                 &AspPlannerThread::gameTimeCallback,
	                                 this));

	RobotMemoryCallbacks.emplace_back(
	  robot_memory->register_trigger(basic::make_document(basic::kvp("relation", "machine")),
	                                 "robmem.planner",
	                                 &AspPlannerThread::machineCallback,
	                                 this));

	RobotMemoryCallbacks.emplace_back(
	  robot_memory->register_trigger(basic::make_document(basic::kvp("relation", "order")),
	                                 "robmem.planner",
	                                 &AspPlannerThread::orderCallback,
	                                 this));

	RobotMemoryCallbacks.emplace_back(
	  robot_memory->register_trigger(basic::make_document(basic::kvp("relation", "ring")),
	                                 "robmem.planner",
	                                 &AspPlannerThread::ringColorCallback,
	                                 this));

	RobotMemoryCallbacks.emplace_back(
	  robot_memory->register_trigger(basic::make_document(basic::kvp("relation", "team-color")),
	                                 "robmem.planner",
	                                 &AspPlannerThread::teamColorCallback,
	                                 this));

	RobotMemoryCallbacks.emplace_back(
	  robot_memory->register_trigger(basic::make_document(basic::kvp("relation", "zones")),
	                                 "robmem.planner",
	                                 &AspPlannerThread::zonesCallback,
	                                 this));

	RobotMemoryCallbacks.emplace_back(
	  robot_memory->register_trigger(basic::make_document(),
	                                 "syncedrobmem.planFeedback",
	                                 &AspPlannerThread::planFeedbackCallback,
	                                 this));

	initPlan();
	initClingo();
	return;
}

constexpr int
constexprStrLen(const char *str)
{
	int size = 0;
	while (*str++) {
		++size;
	} // while ( *str++ )
	return size;
}

void
AspPlannerThread::loop(void)
{
	if (Unsat >= 8) {
		throw fawkes::Exception("The program is infeasible! We have no way to recover!");
	} // if ( Unsat >= 8 )

	{
		MutexLocker                       locker(&WorldMutex);
		const auto                        now(Clock::now());
		static const std::chrono::seconds timeOut(
		  config->get_int(ConfigPrefix + std::string("planner/robot-timeout")));
		for (auto &pair : Robots) {
			auto &info(pair.second);
			if (info.Alive && now - info.LastSeen >= timeOut) {
				logger->log_warn(LoggingComponent, "Robot %s is considered dead.", pair.first.c_str());
				setInterrupt(InterruptSolving::Critical, "Dead robot");
				info.Alive = false;
				if (info.Doing.isValid()) {
					LocationInUse.erase(info.Doing.location());
					info.Doing = {};
					auto &robotPlan(Plan[pair.first]);
					robotPlan.Tasks[robotPlan.FirstNotDone].Begun = false;
					robotPlan.CurrentTask.clear();
				} // if ( info.Doing.isValid() )
			}   // if ( info.Alive && now - info.LastSeen >= timeOut )
		}     // for ( auto& pair : Robots )

		for (auto &pair : Machines) {
			auto &info(pair.second);

			// Broken handling happens in machineCallback().

			if (info.WorkingUntil && info.WorkingUntil <= GameTime) {
				logger->log_info(LoggingComponent,
				                 "Machine %s has finished working on product #%d at %d.",
				                 pair.first.c_str(),
				                 info.Storing.ID,
				                 GameTime);
				info.WorkingUntil = 0;
			} // if ( info.BrokenUntil && info.BrokenUntil <= GameTime )
		}   // for ( auto& pair : Machines )
	}     // Block for iteration over Robots & Machines

	if (GameTime == -1) {
		static bool once = true;
		if (once) {
			MutexLocker worldLocker(&WorldMutex), planLocker(&PlanMutex);
			once = false;

			int totalSum = 0;

			for (const auto &pair : Plan) {
				bool noAdd = false;
				int  sum = 0, last = ReceivedZonesToExplore ? 0 : ExplorationTime, id = 0;
				logger->log_info(LoggingComponent, "Plan & idle time for robot %s", pair.first.c_str());

				constexpr const char *idleFormat = " Idle: %d seconds";
				constexpr const char *failed = " Failed", *notFailed = "";
				constexpr auto        idleLen = constexprStrLen(idleFormat);
				char                  idleString[idleLen + 1 + constexprStrLen("-32766")] = {0};
				//                                             ^^^^^^ longest string
				//                                             value for a short

				for (const auto &task : pair.second.Tasks) {
					if (task.Begin >= ProductionEnd && !noAdd) {
						// Since the end time of the last task didn't end this loop we have
						// to add idle.
						const int idle = ProductionEnd - last;
						logger->log_info(LoggingComponent, "Effektive end game idle: %d", idle);
						sum += idle;
						logger->log_info(LoggingComponent, "==== Game end ====");
						noAdd = true;
					} // if ( task.Begin >= ProductionEnd && !noAdd )

					if (task.Begin > last) {
						const short idle = task.Begin - last;
						std::sprintf(idleString, idleFormat, idle);
						if (!noAdd) {
							sum += idle;
						} // if ( !noAdd )
					}   // if ( task.Begin > last )
					else {
						idleString[0] = 0;
					} // else -> if ( task.Begin > last )
					last = task.End;

					logger->log_info(LoggingComponent,
					                 "Task #%2d: (%-33s, %4d, %4d)%s%s",
					                 ++id,
					                 task.Task.c_str(),
					                 task.Begin,
					                 task.End,
					                 task.Failed ? failed : notFailed,
					                 idleString);

					if (task.End >= ProductionEnd && !noAdd) {
						logger->log_info(LoggingComponent, "==== Game end ====");
						noAdd = true;
					} // if ( task.End >= ProductionEnd && !noAdd )
				}   // for ( const auto& task : pair.second.Tasks )

				logger->log_info(LoggingComponent, "Total idle time for %s: %d", pair.first.c_str(), sum);
				totalSum += sum;
			} // for ( const auto& pair : Plan )

			logger->log_info(LoggingComponent,
			                 "Total idle time: %d, avg. idle time: %zu",
			                 totalSum,
			                 totalSum / Plan.size());

			int id = 0;
			for (const auto &product : Products) {
				bool        found = false, hold = false;
				string_view location;

				for (auto iter = Machines.begin(); iter != Machines.end() && !found; ++iter) {
					if (iter->second.Storing.ID == id) {
						found    = true;
						hold     = false;
						location = iter->first;
					} // if ( iter->second.Storing.ID == id )
				}   // for ( auto iter = Machines.begin(); iter != Machines.end() &&
				// !found; ++iter )

				for (auto iter = Robots.begin(); iter != Robots.end() && !found; ++iter) {
					if (iter->second.Holding.ID == id) {
						found    = true;
						hold     = true;
						location = iter->first;
					} // if ( iter->second.Holding.ID == id )
				}   // for ( auto iter = Robots.begin(); iter != Robots.end() && !found;
				// ++iter )

				logger->log_info(LoggingComponent,
				                 "Product #%d: (%-11s, %-6s, %-6s, %-6s, %-5s) %s %s.",
				                 id++,
				                 product.Base.c_str(),
				                 product.Rings[1].c_str(),
				                 product.Rings[2].c_str(),
				                 product.Rings[3].c_str(),
				                 product.Cap.c_str(),
				                 hold ? "hold   by" : "stored on",
				                 location.data());
			} // for ( const auto& product : Products )
		}   // if ( once )
	}     // if ( GameTime == -1 )
	else {
		loopPlan();
		loopClingo();
	} // else -> if ( GameTime == -1 )
	return;
}

void
AspPlannerThread::finalize(void)
{
	logger->log_info(LoggingComponent, "Finalize ASP Planner");
	for (const auto &callback : RobotMemoryCallbacks) {
		robot_memory->remove_trigger(callback);
	} // for ( const auto& callback : RobotMemoryCallbacks )
	RobotMemoryCallbacks.clear();
	finalizeClingo();
	logger->log_info(LoggingComponent, "ASP Planner finalized");
	return;
}
