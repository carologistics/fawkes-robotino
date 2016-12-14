
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

#include <core/threading/mutex_locker.h>

#include <cstdlib>

using namespace fawkes;

/**
 * @struct RobotInformation
 * @brief Stores information for a robot.
 *
 * @property RobotInformation::LastSeen
 * @brief When did we last hear of the robot.
 *
 * @property RobotInformation::X
 * @brief The reported x coordinate of the robot, when he was added.
 *
 * @property RobotInformation::Y
 * @brief The reported y coordinate of the robot, when he was added.
 *
 * @property RobotInformation::GameTIme
 * @brief The game time, when the robot was added.
 */

/**
 * @struct RingColorInformation
 * @brief Stores information about a ring color.
 *
 * @property RingColorInformation::Color
 * @brief The name of the color.
 *
 * @property RingColorInformation::Machine
 * @brief Which machine serves the color.
 *
 * @property RingColorInformation::Cost
 * @brief How many additional bases are needed to get the color.
 */

/**
 * @struct OrderInformation
 * @brief Stores information about orders.
 *
 * @property OrderInformation::Number
 * @brief The order number.
 *
 * @property OrderInformation::Quantity
 * @brief How many products for the order can be delivered.
 *
 * @property OrderInformation::Base
 * @brief The base color of the ordered product.
 *
 * @property OrderInformation::Cap
 * @brief The cap color of the ordered product.
 *
 * @property OrderInformation::Rings
 * @brief The ring colors of the ordered product, can be "none".
 *
 * @property OrderInformation::DeliveryBegin
 * @brief The begin of the delivery time window.
 *
 * @property OrderInformation::DeliveryEnd
 * @brief The end of the delivery time window.
 *
 * @property OrderInformation::GameTime
 * @brief The game time, when we received the order.
 */

/** @class AspPlanerThread "asp_planer_thread.h"
 * The thread to start and control the ASP planer.
 *
 * @property AspPlanerThread::RobotMemoryCallbacks
 * @brief Contains all registered callbacks in the robot memory.
 *
 * @property AspPlanerThread::Unsat
 * @brief The program was unsatisfiable.
 *
 * @property AspPlanerThread::RobotsMutex
 * @brief The mutex for the robots information.
 *
 * @property AspPlanerThread::RobotInformations
 * @brief The robot information in a lookup table.
 *
 * @property AspPlanerThread::RobotTaskBegin
 * @brief Saves when a robot has begun a task, so we can fix this in the asp program.
 *
 * @property AspPlanerThread::RobotTaskUpdate
 * @brief Saves when a robot updates a task time estimation, so we can fix this in the asp program.
 *
 * @property AspPlanerThread::TaskSuccess
 * @brief Saves when a task was successfully executed.
 */

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
		MutexLocker locker(&RobotsMutex);
		if ( !RobotInformations.count(name) )
		{
			newRobot = true;
		} //if ( !RobotInformations.count(name) )
		const auto& time(object["last-seen"].Array());
		auto& info = RobotInformations[name];
		if ( newRobot )
		{
			info = {Time(time.at(0).Long(), time.at(1).Long()),
				static_cast<float>(object["x"].Double()), static_cast<float>(object["y"].Double()), GameTime};
			logger->log_info(LoggingComponent, "New robot %s detected.", name.c_str());
			newTeamMate(name, info);
		} //if ( newRobot )
		else
		{
			info.LastSeen = Time(Time(time.at(0).Long(), time.at(1).Long()));
		} //else -> if ( newRobot )
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
		OrderInformation info{number, quantity, base, cap, {ring1, ring2, ring3}, delBegin, delEnd,
			std::max(GameTime, ExplorationTime)};
		if ( RingColors.size() == 4 )
		{
			//Do only spawn tasks when we have the ring infos.
			addOrder(info);
		} //if ( RingColors.size() > 4 )

		Orders.emplace_back(std::move(info));

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
		RingColorInformation info{color, machine, cost};
		setRingColor(info);
		RingColors.emplace_back(std::move(info));

		if ( RingColors.size() == 4 )
		{
			//We have the last ring info, spawn orders we have received until now.
			for ( const auto& order : Orders )
			{
				addOrder(order);
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
		const auto zones(object["zones"].Array());
		char *unused;
		for ( const auto& zone : zones )
		{
			//+1 to get rid of the Z.
			addZoneToExplore(std::strtol(zone.String().c_str() + 1, &unused, 10));
		} //for ( const auto& zone : zones )
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

/** Constructor. */
AspPlanerThread::AspPlanerThread(void) : Thread("AspPlanerThread", Thread::OPMODE_CONTINUOUS),
		ASPAspect("ASPPlaner", "ASP-Planer"),
		LoggingComponent("ASP-Planer-Thread"), ConfigPrefix("/asp-agent/"), TeamColor(nullptr), MoreModels(false),
		ExplorationTime(0), MaxOrders(0), MaxQuantity(0), LookAhaed(0), GameTime(0), Horizon(0), Past(0),
		MachinesFound(0), StillNeedExploring(true), CompleteRestart(false), TimeResolution(1), MaxDriveDuration(0),
		PlanElements(0), Unsat(false), UpdateNavgraphDistances(false),
		Interrupt(InterruptSolving::Not), SentCancel(false)
{
	//We don't expect more than 3 robots.
	Robots.reserve(3);
	constructClingo();
	return;
}

/** Destructor. */
AspPlanerThread::~AspPlanerThread(void)
{
	return;
}

void
AspPlanerThread::init(void)
{
	logger->log_info(LoggingComponent, "Initialize ASP Planer");
	const auto prefixLen = std::strlen(ConfigPrefix);
	char buffer[prefixLen + 20];
	const auto suffix = buffer + prefixLen;
	std::strcpy(buffer, ConfigPrefix);

	std::strcpy(suffix, "exploration-time");
	ExplorationTime = config->get_uint(buffer);

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
		MutexLocker locker(&RobotsMutex);
		const auto now(clock->now());
		static const Time timeOut(10, 0);
		auto iter = RobotInformations.begin();
		while ( iter != RobotInformations.end() )
		{
			if ( now - iter->second.LastSeen >= timeOut )
			{
				logger->log_warn(LoggingComponent, "Robot %s is considered dead.", iter->first.c_str());
				releaseExternals(iter->second);
				deadTeamMate(iter->first);
				iter = RobotInformations.erase(iter);
			} //if ( now - iter->second.LastSeen >= timeOut )
			else
			{
				++iter;
			} //else -> if ( now - iter->second.LastSeen >= timeOut )
		} //while ( iter != RobotInformations.end() )
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

//bool
//AspAgentThread::prepare_finalize_user()
//{
//	logger->log_info(LoggingComponent, "Prepare Finalize ASP Planer");
//	return true;
//}

