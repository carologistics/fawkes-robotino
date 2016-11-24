
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

using namespace fawkes;

/**
 * @struct RobotInformation
 * @brief Stores information for a robot.
 *
 * @property RobotInformation::LastSeen
 * @brief When did we last hear of the robot.
 *
 * @property RobotInformation::X
 * @brief The reported x coordinate of the robot.
 *
 * @property RobotInformation::Y
 * @brief The reported y coordinate of the robot.
 */

/** @class AspPlanerThread "asp_planer_thread.h"
 * The thread to start and control the ASP planer.
 *
 * @property AspPlanerThread::RobotMemoryCallbacks
 * @brief Contains all registered callbacks in the robot memory.
 *
 * @property AspPlanerThread::Robots
 * @brief The robot information in a lookup table.
 */

/**
 * @brief Gets called, when a beacon is received, updates the robot information.
 * @param[in] document The DB document.
 */
void AspPlanerThread::beaconCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const std::string& name(object["name"].String());
		bool newRobot = false;
		MutexLocker locker(&RobotsMutex);
		if ( !Robots.count(name) )
		{
			newRobot = true;
		} //if ( !Robots.count(name) )
		const auto& time(object["last-seen"].Array());
		Robots[name] = {Time(time.at(0).Long(), time.at(1).Long()), object["x"].Double(), object["y"].Double()};
		if ( newRobot )
		{
			logger->log_info(LoggingComponent, "New robot %s detected.", name.c_str());
			newTeamMate(name);
		} //if ( newRobot )
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
 * @brief Gets called, when the team color is changed.
 * @param[in] document The document with the new color.
 */
void AspPlanerThread::teamColorCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const std::string& color(object["values"].Array().at(0).String());
		if ( TeamColorSet )
		{
			if ( color == "nil" )
			{
				TeamColorSet = false;
				logger->log_info(LoggingComponent, "Unsetting Team-Color.");
			} //if ( color == "nil" )
			else
			{
				logger->log_info(LoggingComponent, "Changing Team-Color to %s.", color.c_str());
			} //else -> if ( color == "nil" )
			unsetTeam();
		} //if ( TeamColorSet )
		else
		{
			logger->log_info(LoggingComponent, "Setting Team-Color to %s.", color.c_str());
			TeamColorSet = true;
		} //else -> if ( TeamColorSet )
		if ( color != "nil" )
		{
			setTeam(color == "CYAN");
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

/** Constructor. */
AspPlanerThread::AspPlanerThread(void) : Thread("AspPlanerThread", Thread::OPMODE_WAITFORWAKEUP),
		BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK), ASPAspect("ASPPlaner", "ASP-Planer"),
		LoggingComponent("ASP-Planer-Thread"), ConfigPrefix("/asp-agent/"), TeamColorSet(false), MoreModels(false),
		ExplorationTime(0), LastTick(0), GameTime(0), LastGameTime(0), Horizon(0)
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
AspPlanerThread::init()
{
	logger->log_info(LoggingComponent, "Initialize ASP Planer");
	const auto prefixLen = std::strlen(ConfigPrefix);
	char buffer[prefixLen + 20];
	const auto suffix = buffer + prefixLen;
	std::strcpy(buffer, ConfigPrefix);

	std::strcpy(suffix, "exploration-time");
	ExplorationTime = config->get_uint(buffer);

	RobotMemoryCallbacks.reserve(2);

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(R"({"relation": "active-robot", "name": {$ne: "RefBox"}})"), "robmem.planer",
		&AspPlanerThread::beaconCallback, this));

	RobotMemoryCallbacks.emplace_back(robot_memory->register_trigger(
		mongo::Query(R"({"relation": "team-color"})"), "robmem.planer",
		&AspPlanerThread::teamColorCallback, this));

	initClingo();
	return;
}

void
AspPlanerThread::loop()
{
	{
		MutexLocker locker(&RobotsMutex);
		const auto now(clock->now());
		static const Time timeOut(10, 0);
		auto iter = Robots.begin();
		while ( iter != Robots.end() )
		{
			if ( now - iter->second.LastSeen >= timeOut )
			{
				logger->log_warn(LoggingComponent, "Robot %s is considered dead.", iter->first.c_str());
				deadTeamMate(iter->first);
				iter = Robots.erase(iter);
			} //if ( now - iter->second.LastSeen >= timeOut )
			else
			{
				++iter;
			} //else -> if ( now - iter->second.LastSeen >= timeOut )
		} //while ( iter != Robots.end() )
	} //Block for iteration over Robots
	loopClingo();
	return;
}

void
AspPlanerThread::finalize()
{
	logger->log_info(LoggingComponent, "Finalize ASP Planer");
	for ( const auto& callback : RobotMemoryCallbacks )
	{
		robot_memory->remove_trigger(callback);
	} //for ( const auto& callback : RobotMemoryCallbacks )
	RobotMemoryCallbacks.clear();
	finalizeClingo();
	return;
}

//bool
//AspAgentThread::prepare_finalize_user()
//{
//	logger->log_info(LoggingComponent, "Prepare Finalize ASP Planer");
//	return true;
//}

