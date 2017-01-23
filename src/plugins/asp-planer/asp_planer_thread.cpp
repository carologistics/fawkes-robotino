
/***************************************************************************
 *  aps_planer_thread.cpp -  ASP-based planer main thread
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

#include "asp_planer_thread.h"
#include "asp_planer_externals.h"

#include <core/threading/mutex_locker.h>

#include <algorithm>
#include <cstdlib>
#include <string>

using namespace fawkes;

/**
 * @brief Transforms the real time to ASP time steps.
 * @param[in] realGameTime The real game time in seconds.
 * @return The ASP time units.
 */
unsigned int
AspPlanerThread::realGameTimeToAspGameTime(const unsigned int realGameTime) const noexcept
{
	if ( realGameTime == 0 )
	{
		return 0;
	} //if ( realGameTime == 0 )
	return std::max(1u, realGameTime / TimeResolution + ((realGameTime % TimeResolution) * 2 >= TimeResolution ? 1 : 0));
}

/**
 * @brief Transforms the ASP time steps to real time.
 * @param[in] aspGameTime The ASP time units.
 * @return The real game time in seconds.
 */
unsigned int
AspPlanerThread::aspGameTimeToRealGameTime(const unsigned int aspGameTime) const noexcept
{
	return aspGameTime * TimeResolution;
}

/**
 * @brief Gets called, when a beacon is received, updates the robot information.
 * @param[in] document The DB document.
 */
void
AspPlanerThread::beaconCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const std::string name(object["name"].String());
		bool newRobot = false;
		MutexLocker locker(&WorldMutex);
		if ( !Robots.count(name) )
		{
			newRobot = true;
		} //if ( !Robots.count(name) )
		const auto& time(object["last-seen"].Array());
		auto& info = Robots[name];

		if ( newRobot )
		{
			logger->log_info(LoggingComponent, "New robot %s detected.", name.c_str());
			setInterrupt(InterruptSolving::Critical);
			info.AliveExternal = generateAliveExternal(name);
		} //if ( newRobot )
		info.LastSeen = Time(time.at(0).Long(), time.at(1).Long());
		info.Alive = true;
		info.X = static_cast<float>(object["x"].Double());
		info.Y = static_cast<float>(object["y"].Double());
	} //try
	catch ( const std::exception& e )
	{
		logger->log_error(LoggingComponent, "Exception while updating robot information: %s\n%s", e.what(),
			document.toString().c_str());
	} //catch ( const std::exception& e )
	catch ( ... )
	{
		logger->log_error(LoggingComponent, "Exception while updating robot information.\n%s",
			document.toString().c_str());
	} //catch ( ... )
	return;
}

/**
 * @brief Gets called, when a game state is received, updates the internal game-time.
 * @param[in] document The DB document.
 */
void
AspPlanerThread::gameTimeCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const std::string phase(object["phase"].String());
		const unsigned int gameTime(object["time"].Long());
		MutexLocker locker(&WorldMutex);
		if ( phase == "EXPLORATION" )
		{
			GameTime = gameTime;
		} //if ( phase == "EXPLORATION" )
		else if ( phase == "PRODUCTION" )
		{
			GameTime = ExplorationTime + gameTime;
		} //else if ( phase == "PRODUCTION" )
	} //try
	catch ( const std::exception& e )
	{
		logger->log_error(LoggingComponent, "Exception while updating game time: %s\n%s", e.what(),
			document.toString().c_str());
	} //catch ( const std::exception& e )
	catch ( ... )
	{
		logger->log_error(LoggingComponent, "Exception while updating game time.\n%s",
			document.toString().c_str());
	} //catch ( ... )
	return;
}

/**
 * @brief Gets called if we got a new order.
 * @param[in] document The information about the order.
 */
void
AspPlanerThread::orderCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const unsigned int number(object["number"].Long());
		const unsigned int quantity(object["quantity"].Long());
		const std::string base(object["base"].String());
		const std::string cap(object["cap"].String());
		const auto rings(object["rings"].Array());
		const std::string ring1(rings.size() >= 1 ? rings[0].String() : "none");
		const std::string ring2(rings.size() >= 2 ? rings[1].String() : "none");
		const std::string ring3(rings.size() >= 3 ? rings[2].String() : "none");
		const unsigned int delBegin(object["begin"].Long() + ExplorationTime);
		const unsigned int delEnd(object["end"].Long() + ExplorationTime);

		MutexLocker locker(&WorldMutex);
		Orders.emplace_back(OrderInformation{number, quantity, base, cap, ring1, ring2, ring3, delBegin, delEnd});

		if ( RingColors.size() == 4 )
		{
			//Do only spawn tasks when we have the ring infos.
			addOrderToASP(Orders.back());
		} //if ( RingColors.size() > 4 )

		if ( number > MaxOrders )
		{
			logger->log_error(LoggingComponent, "We expect no higher order numbers than %d, but got %d! This order "
				"will not be considered by the ASP program!", MaxOrders, number);
		} //if ( number > MaxOrders )
		if ( quantity > MaxQuantity )
		{
			logger->log_error(LoggingComponent, "We expect no higher quantities for orders than %d, but got %d! This "
				"order will not be considered by the ASP program!", MaxQuantity, quantity);
		} //if ( quantity > MaxQuantity )
	} //try
	catch ( const std::exception& e )
	{
		logger->log_error(LoggingComponent, "Exception while extracting an order: %s\n%s", e.what(),
			document.toString().c_str());
	} //catch ( const std::exception& e )
	catch ( ... )
	{
		logger->log_error(LoggingComponent, "Exception while extracting an order.\n%s",
			document.toString().c_str());
	} //catch ( ... )
	return;
}

/**
 * @brief Gets called if we know all we need to know about a ring color.
 * @param[in] document The information about the color.
 */
void
AspPlanerThread::ringColorCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const std::string color(object["color"].String());
		const unsigned int cost(object["cost"].Long());
		const std::string machine(object["machine"].String().substr(2));

		MutexLocker locker(&WorldMutex);
		RingColors.emplace_back(RingColorInformation{color, machine, cost});

		addRingColorToASP(RingColors.back());

		if ( RingColors.size() == 4 )
		{
			//We have the last ring info, spawn orders we have received until now.
			for ( const auto& order : Orders )
			{
				addOrderToASP(order);
			} //for ( const auto& order : Orders )
		} //if ( RingColors.size() == 4 )
	} //try
	catch ( const std::exception& e )
	{
		logger->log_error(LoggingComponent, "Exception while setting ring color: %s\n%s", e.what(),
			document.toString().c_str());
	} //catch ( const std::exception& e )
	catch ( ... )
	{
		logger->log_error(LoggingComponent, "Exception while setting ring color.\n%s",
			document.toString().c_str());
	} //catch ( ... )
	return;
}

/**
 * @brief Gets called, when the team color is changed.
 * @param[in] document The document with the new color.
 */
void
AspPlanerThread::teamColorCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const std::string color(object["values"].Array().at(0).String());
		if ( TeamColor )
		{
			if ( color == "nil" )
			{
				TeamColor = nullptr;
				logger->log_info(LoggingComponent, "Unsetting Team-Color.");
			} //if ( color == "nil" )
			else
			{
				logger->log_info(LoggingComponent, "Changing Team-Color to %s.", color.c_str());
			} //else -> if ( color == "nil" )
			unsetTeam();
		} //if ( TeamColor )
		else
		{
			logger->log_info(LoggingComponent, "Setting Team-Color to %s.", color.c_str());
		} //else -> if ( TeamColor )

		if ( color != "nil" )
		{
			TeamColor = color == "CYAN" ? "C" : "M";
			setTeam();
		} //if ( color != "nil" )
	} //try
	catch ( const std::exception& e )
	{
		logger->log_error(LoggingComponent, "Exception while updating the team color: %s\n%s", e.what(),
			document.toString().c_str());
	} //catch ( const std::exception& e )
	catch ( ... )
	{
		logger->log_error(LoggingComponent, "Exception while updating the team color.\n%s",
			document.toString().c_str());
	} //catch ( ... )
	return;
}

/**
 * @brief Gets called, when the zones to explore are set.
 * @param[in] document The document with the zones.
 */
void
AspPlanerThread::zonesCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const auto zonesArray(object["zones"].Array());
		std::vector<unsigned int> zones;
		zones.reserve(zonesArray.size());
		std::transform(std::begin(zonesArray), std::end(zonesArray), std::back_inserter(zones),
			[](const mongo::BSONElement& zone)
			{
				return std::stoi(zone.String().substr(1));
			});
		const auto begin = zones.begin();
		auto end = zones.end();
		MutexLocker locker(&WorldMutex);
		for ( auto zone = 1u; zone <= 24u; ++zone )
		{
			auto iter = std::find(begin, end, zone);
			if ( iter == end )
			{
				releaseZone(zone, false);
			} //if ( iter == end )
			else
			{
				ZonesToExplore.push_back(zone);
				addZoneToExplore(zone);
				/* Move the found zone to the end, and move the end iterator one to the left. So the std::find has to
				 * search a smaller range. */
				std::swap(*iter, *--end);
			} //else -> if ( iter == end )
		} //for ( auto zone = 1u; zone <= 24u; ++zone )
		fillNavgraphNodesForASP(false);
	} //try
	catch ( const std::exception& e )
	{
		logger->log_error(LoggingComponent, "Exception while extracting the zones to explore: %s\n%s", e.what(),
			document.toString().c_str());
	} //catch ( const std::exception& e )
	catch ( ... )
	{
		logger->log_error(LoggingComponent, "Exception while extracting the zones to explore.\n%s",
			document.toString().c_str());
	} //catch ( ... )
	return;
}

/**
 * @brief Constructor.
 */
AspPlanerThread::AspPlanerThread(void) : Thread("AspPlanerThread", Thread::OPMODE_CONTINUOUS),
		ASPAspect("ASPPlaner", "ASP-Planer"),
		LoggingComponent("ASP-Planer-Thread"), ConfigPrefix("/asp-agent/"), TeamColor(nullptr),
		Unsat(false),
		//Config
		ExplorationTime(0), DeliverProductTaskDuration(0), FetchProductTaskDuration(0), LookAhaed(0),
		MaxDriveDuration(0), MaxOrders(0), MaxProducts(0), MaxQuantity(0), MaxTaskDuration(0), PrepareCSTaskDuration(0),
		TimeResolution(0),
		//Worldmodel
		GameTime(0),
		//Distances
		UpdateNavgraphDistances(true),
		//Requests
		Interrupt(InterruptSolving::Not), SentCancel(false),
		//Solving, loop intern
		ProgramGrounded(false), ProductionStarted(false),
		//Solving
		NewSymbols(false),
		//Plan
		StartSolvingGameTime(0)
		/*Horizon(0), Past(0),
		MachinesFound(0), StillNeedExploring(true), CompleteRestart(false),
		PlanElements(0), UpdateNavgraphDistances(false),*/
{
	return;
}

/**
 * @brief Destructor.
 */
AspPlanerThread::~AspPlanerThread(void)
{
	return;
}

/**
 * @brief Initliaizes the robmem callbacks and world model. Also calls the specialized inits.
 */
void
AspPlanerThread::init(void)
{
	logger->log_info(LoggingComponent, "Initialize ASP Planer");
	loadConfig();
	Orders.reserve(MaxOrders);
	RingColors.reserve(4);
	Robots.reserve(PossibleRobots.size());
	ZonesToExplore.reserve(12);

	RobotMemoryCallbacks.reserve(7);

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(R"({"relation": "active-robot", "name": {$ne: "RefBox"}})"), "robmem.planer",
		&AspPlanerThread::beaconCallback, this));

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(R"({"relation": "game-time"})"), "robmem.planer",
		&AspPlanerThread::gameTimeCallback, this));

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(R"({"relation": "order"})"), "robmem.planer",
		&AspPlanerThread::orderCallback, this));

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(R"({"relation": "ring"})"), "robmem.planer",
		&AspPlanerThread::ringColorCallback, this));

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(R"({"relation": "team-color"})"), "robmem.planer",
		&AspPlanerThread::teamColorCallback, this));

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(R"({"relation": "zones"})"), "robmem.planer",
		&AspPlanerThread::zonesCallback, this));

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(), "syncedrobmem.planFeedback", &AspPlanerThread::planFeedbackCallback, this));

	initPlan();
	initClingo();
	return;
}

void
AspPlanerThread::loop(void)
{
	if ( Unsat )
	{
		throw fawkes::Exception("The program is infeasable! We have no way to recover!");
	} //if ( Unsat )

	{
		MutexLocker locker(&WorldMutex);
		const auto now(clock->now());
		static const Time timeOut(config->get_int(ConfigPrefix + std::string("planer/robot-timeout")), 0);
		for ( auto& pair : Robots )
		{
			auto& info(pair.second);
			if ( info.Alive && now - info.LastSeen >= timeOut )
			{
				logger->log_warn(LoggingComponent, "Robot %s is considered dead.", pair.first.c_str());
				setInterrupt(InterruptSolving::Critical);
				info.Alive = false;
			} //if ( info.Alive && now - info.LastSeen >= timeOut )
		} //for ( auto& pair : Robots )
	} //Block for iteration over Robots

	loopPlan();
	loopClingo();
	return;
}

void
AspPlanerThread::finalize(void)
{
	logger->log_info(LoggingComponent, "Finalize ASP Planer");
	for ( const auto& callback : RobotMemoryCallbacks )
	{
		robot_memory->remove_trigger(callback);
	} //for ( const auto& callback : RobotMemoryCallbacks )
	RobotMemoryCallbacks.clear();
	finalizeClingo();
	logger->log_info(LoggingComponent, "ASP Planer finalized");
	return;
}

