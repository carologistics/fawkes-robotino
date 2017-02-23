/***************************************************************************
 *  asp_planer_plan.cpp - ASP-based planer plugin plan composition
 *
 *  Created on Mon Dec 05 19:32:02 2016
 *  Copyright (C) 2016 by Björn Schäpers
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

#include "asp_planer_thread.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <type_traits>

#include <core/threading/mutex_locker.h>
#include <plugins/robot-memory/robot_memory.h>

using fawkes::MutexLocker;

/**
 * @brief Initalized plan elements.
 */
void
AspPlanerThread::initPlan(void)
{
	//Clear old plan, if in db.
	robot_memory->drop_collection("syncedrobmem.plan");
	return;
}

/**
 * @brief Handles everything concerning the plan in the loop.
 */
void
AspPlanerThread::loopPlan(void)
{
	MutexLocker solvingLocker(&SolvingMutex);
	if ( !NewSymbols )
	{
		return;
	} //if ( !NewSymbols )

	static const std::chrono::milliseconds planThreshold(config->get_int("/asp-agent/planer/plan-extraction-delay"));
	if ( Clock::now() - LastModel < planThreshold )
	{
		//The last model is fresh, maybe there is a new and better in the next loop, wait a short time.
		return;
	} //if ( Clock::now() - LastModel < planThreshold )

	if ( LastModel < SolvingStarted )
	{
		//The last model is based on old information, do not use it! Without this there were wrong plans!
		return;
	} //if ( LastModel < SolvingStarted )

	/* There is a new model and it is old enough, start with the plan extraction. Since we have no guarantees about the
	 * ordering in the model we use a map to assemble the (robot, task, begin, end) tuples. */
	MutexLocker planLocker(&PlanMutex);

	//Make this map static, so we do not release the needed memory everytime.
	static std::unordered_map<std::pair<std::string, std::string>, std::pair<int, int>> map;
	map.clear();

	PlanGameTime = StartSolvingGameTime;
	for ( const auto& symbol : Symbols )
	{
		const string_view name(symbol.name());
		const bool begin = name == "begin", end = name == "end";
		if ( begin || end )
		{
			//Assumes begin(robot, task, time) and end(robot, task, time).
			const decltype(auto) args(symbol.arguments());
			//If not in the map until now it will add (robot, task, 0, 0).
			auto& pair = map[{args[0].string(), args[1].to_string()}];
			(begin ? pair.first : pair.second) = aspGameTimeToRealGameTime(args[2].number()) + PlanGameTime;
		} //if ( begin || end )
	} //for ( const auto& symbol : Symbols )

	NewSymbols = false;
	solvingLocker.unlock();
	LastPlan = Clock::now();

	//Some helper functions to handle plan elements.
	//Used to sort the vector and find elements.
	auto planBegin = [](const BasicPlanElement& e1, const BasicPlanElement& e2) noexcept
		{
			return e1.Begin < e2.Begin;
		};
	//If two plan elements refer to the same task.
	auto sameTask = [](const BasicPlanElement& e1, const BasicPlanElement& e2) noexcept
		{
			return e1.Task == e2.Task;
		};
	//Compares the begin and end time point and if their difference is above a given threshold.
	//This is only valid if both elements are about the same task.
	auto needsUpdate = [this,sameTask](const BasicPlanElement& e1, const BasicPlanElement& e2) noexcept
		{
			assert(sameTask(e1, e2));
			static_assert(std::is_signed<decltype(e1.Begin)>::value,
				"For unsigned values std::abs doesn't make any sense.");
			const auto threshold = TimeResolution / 2;
			if ( e2.Begin == 0 )
			{
				/* This is a task, which was running when the solving started, since we never have a begin() the value
				 * is not set. So just compare the end values. */
				return std::abs(e1.End - e2.End) > threshold;
			} //if ( e2.Begin == 0 )
			return std::abs(e1.Begin - e2.Begin) > threshold || std::abs(e1.End - e2.End) > threshold;
		};

	//Assemble a plan only based on the information from clingo.
	static std::unordered_map<std::string, std::vector<BasicPlanElement>> tempPlan;
	for ( const auto& robot : PossibleRobots )
	{
		tempPlan[robot].clear();
	} //for ( const auto& robot : PossibleRobots )

	logger->log_info(LoggingComponent, "Extracted plan size: %d", map.size());
	for ( const auto& pair : map )
	{
		const auto& robotName(pair.first.first);
		logger->log_info(LoggingComponent, "Plan element: (%s, %s, %d, %d)",
			robotName.c_str(), pair.first.second.c_str(), pair.second.first, pair.second.second);
		auto& robotTempPlan(tempPlan[robotName]);
		BasicPlanElement e{pair.first.second, pair.second.first, pair.second.second};
		auto iter = robotTempPlan.insert(std::lower_bound(robotTempPlan.begin(), robotTempPlan.end(), e, planBegin),
			std::move(e));
		if ( iter->Begin == 0 )
		{
			//The first entry is a already started task, fix its begin time!
			auto& robotPlan(Plan[robotName]);
			iter->Begin = robotPlan.Tasks[robotPlan.FirstNotDone].Begin;
		} //if ( iter->Begin == 0 )
	} //for ( const auto& pair : map )

	for ( const auto& pair : tempPlan )
	{
		const auto& robotName(pair.first);
		auto& robotPlan(Plan[robotName].Tasks);
		const auto& tempRobotPlan(pair.second);

		logger->log_info(LoggingComponent, "Update plan for %s.", robotName.c_str());

		//Cache the end iterator of the old robot plan.
		auto planEnd = robotPlan.end();
		//Show at the first task the robot has not yet started.
		auto notBegun = std::find_if(robotPlan.begin(), planEnd, [](const PlanElement& e) noexcept { return !e.Begun;});
		/* Depending on the size of the new robot plan we get the iterator where we think we have to update.
		 * If there is a new plan, just look where the first entry would land.
		 * If there is no plan, there is no first entry, so just copy the first not done task, because we do not want
		 * to interrupt this. */
		auto planIter = tempRobotPlan.empty() ? notBegun :
			std::lower_bound(robotPlan.begin(), planEnd, tempRobotPlan[0], planBegin);

		//The two iterators of the temp plan.
		auto tempIter = tempRobotPlan.begin();
		const auto tempEnd = tempRobotPlan.end();

		//Compute the task index based on the used plan iterator.
		int index = planIter - robotPlan.begin();

		//Helping lambda to remove all tasks which index is greater or equal to the variable index.
		auto removeAllFromIndexOn = [&](void)
			{
				if ( planIter == planEnd )
				{
					return planIter;
				} //if ( planIter == planEnd )

				logger->log_warn(LoggingComponent, "Remove all %lu robot plan elements from (%s,%s,%d,%d) on.",
					planEnd - planIter, robotName.c_str(), planIter->Task.c_str(), planIter->Begin, planIter->End);
				const decltype(index) size(robotPlan.size());
				for ( auto i = index; i < size; ++i )
				{
					removeFromPlanDB(robotName, i);
				} //for ( auto i = index; i < size; ++i )
				return robotPlan.erase(planIter, planEnd);
			};

		if ( notBegun < planIter )
		{
			/* There is at least one element that is before the point we want to add the new elements and not yet begun.
			 * We have to remove this from the plan, because the new plans for the other robots do not consider this, it
			 * could lead to conflicts, i.e. two robots trying to do the same task. Because we make so rigorous changes
			 * we remove all not started tasks, they will be addes on demand after wards. */
			index = notBegun - robotPlan.begin();
			planIter = notBegun;
			planIter = planEnd = removeAllFromIndexOn();
		} //if ( notBegun < planIter )

		//Flag if we should continue this loop with the next entry after the following loop.
		bool nextRobot = false;

		//There is an entry in our old plan, which competes with an entry of the new plan for the same index.
		while ( planIter != planEnd && tempIter != tempEnd )
		{
			if ( sameTask(*planIter, *tempIter) )
			{
				//Everything is fine, we have the same task, maybe only update the timing.
				if ( needsUpdate(*planIter, *tempIter) )
				{
					logger->log_info(LoggingComponent, "Update time for (%s,%s,%d,%d) to (%d,%d)", robotName.c_str(),
						planIter->Task.c_str(), planIter->Begin, planIter->End, tempIter->Begin, tempIter->End);
					if ( planIter->Begun )
					{
						logger->log_warn(LoggingComponent, "The task is already started!");
					} //if ( planIter->Begun )
					planIter->updateTime(*tempIter);
					updatePlanTiming(robotName, index, *planIter);
				} //if ( needsUpdate(*planIter, *tempIter) )
			} //if ( sameTask(*planIter, *tempIter) )
			else
			{
				//A different task, handle differently for already started tasks.
				const auto robotCStr = robotName.c_str();
				if ( planIter->Begun )
				{
					logger->log_warn(LoggingComponent,
						"Should change started task %s from robot %s to %s. Restart solving!",
						planIter->Task.c_str(), robotCStr, tempIter->Task.c_str());

					/* Remove all following tasks from the robots plan, because they may conflict with the other new
					 * robots plans. This task maybe do the same, but this should be detected when the other robots
					 * want to start their tasks. */
					++planIter;
					++index;
					removeAllFromIndexOn();

					nextRobot = true;
					setInterrupt(InterruptSolving::Critical);
					break;
				} //if ( planIter->Begun )

				logger->log_info(LoggingComponent, "Change from (%s,%s,%d,%d) to (%s,%s,%d,%d).",
					robotCStr, planIter->Task.c_str(), planIter->Begin, planIter->End,
					robotCStr, tempIter->Task.c_str(), tempIter->Begin, tempIter->End);
				planIter->updateTimeAndTask(*tempIter);
				updatePlan(robotName, index, *planIter);
			} //else -> if ( && sameTask(*planIter, *tempIter) )
			++planIter;
			++tempIter;
			++index;
		} //while ( planIter != planEnd && tempIter != tempEnd )

		if ( nextRobot )
		{
			continue;
		} //if ( nextRobot )

		if ( tempIter != tempEnd )
		{
			//We have more tasks in our temp plan than in the deployed plan left.
			robotPlan.reserve(robotPlan.size() + tempEnd - tempIter);
			std::copy(tempIter, tempEnd, std::back_inserter(robotPlan));
			do //while ( ++index < static_cast<int>(robotPlan.size()) )
			{
				insertPlanElement(robotName, index, robotPlan[index]);
			} while ( ++index < static_cast<int>(robotPlan.size()) );
		} //if ( tempIter != tempEnd )
		else if ( planIter != planEnd )
		{
			//We have less tasks in our temp plan than in the deployed plan left.
			if ( planIter->Begun )
			{
				//Deleting a task which is already started seems not to be the best idea, restart solving.
				logger->log_warn(LoggingComponent, "Should delete started task %s for robot %s! Restart solving.",
					planIter->Task.c_str(), robotName.c_str());
				setInterrupt(InterruptSolving::Critical);
				//But remove all tasks after this.
				++planIter;
				++index;
			} //if ( planIter->Begun )

			removeAllFromIndexOn();
		} //else if ( planIter != planEnd )
	} //for ( const auto& pair : Plan )

	logger->log_info(LoggingComponent, "Plan updating done.");
	return;
}

/**
 * @brief Transforms the task string from ASP syntax to CLIPS syntax.
 * @param[in] string The string to transform.
 * @return The transformed string.
 */
static std::string
taskASPtoCLIPS(std::string string)
{
	std::transform(string.begin(), string.end(), string.begin(),
		[](const char c) noexcept {
			switch ( c )
			{
				case ',' : return ' ';
				case '"' : return '\'';
			} //switch ( c )
			return c;
		});
	return string;
}

/**
 * @brief Transforms the task string from CLIPS syntax to ASP syntax.
 * @param[in] string The string to transform.
 * @return The transformed string.
 */
static std::string
taskCLIPStoASP(std::string string)
{
	std::transform(string.begin(), string.end(), string.begin(),
		[](const char c) noexcept {
			switch ( c )
			{
				case ' '  : return ',';
				case '\'' : return '"';
			} //switch ( c )
			return c;
		});
	return string;
}

/**
 * @brief Helper function to create the query.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element.
 * @return The MongoDB query.
 */
static mongo::BSONObj
createQuery(const std::string& robot, const int elementIndex)
{
	mongo::BSONObjBuilder builder;
	builder.append("relation", "planElement").append("robot", robot).append("index", elementIndex);
	return builder.obj();
}

/**
 * @brief Heper function to create a BSON object out of a PlanElement.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element, used for the ordering on the executive. Is ignored if set to -1.
 * @param[in] element The element.
 * @return The object.
 */
static mongo::BSONObj
createObject(const std::string& robot, const int elementIndex, const PlanElement& element)
{
	mongo::BSONObjBuilder builder;
	builder.append("relation", "planElement").append("robot", robot).
		append("task", "\"" + taskASPtoCLIPS(element.Task) + "\"").
		append("begin", element.Begin).append("end", element.End);
	if ( elementIndex >= 0 )
	{
		 builder.append("index", elementIndex);
	} //if ( elementIndex >= 0 )
	return builder.obj();
}

/**
 * @brief Inserts a new element in the plan.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element, used for the ordering on the executive.
 * @param[in] element The element.
 */
void
AspPlanerThread::insertPlanElement(const std::string& robot, const int elementIndex, const PlanElement& element)
{
	robot_memory->insert(createObject(robot, elementIndex, element), "syncedrobmem.plan");
	return;
}

/**
 * @brief Updates the plan element, actually replaces it.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element, used for the ordering on the executive.
 * @param[in] element The element.
 */
void
AspPlanerThread::updatePlan(const std::string& robot, const int elementIndex, const PlanElement& element)
{
	robot_memory->update(createQuery(robot, elementIndex), createObject(robot, elementIndex, element),
		"syncedrobmem.plan", true);
	return;
}

/**
 * @brief Updates the plan elements time in the robot memory.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element, used for the ordering on the executive.
 * @param[in] element The element.
 */
void
AspPlanerThread::updatePlanTiming(const std::string& robot, const int elementIndex, const PlanElement& element)
{
	/* The idea was only to update the time, but this either don't work, or my query wasn't good enough, so call the
	 * general update method. */
	updatePlan(robot, elementIndex, element);
	return;
}

/**
 * @brief Removes a plan element from the robot memory.
 * @param[in] robot For which robot the plan element is.
 * @param[in] element The element.
 */
void
AspPlanerThread::removeFromPlanDB(const std::string& robot, const int elementIndex)
{
	robot_memory->remove(createQuery(robot, elementIndex), "syncedrobmem.plan");
	return;
}

/**
 * @brief Tells the robot to stop its current task and removes its plan, because the plan is corrupted.
 * @param[in] robot The robots name.
 * @note The plan lock has to be hold.
 */
void
AspPlanerThread::tellRobotToStop(const std::string& robot)
{
	mongo::BSONObjBuilder builder;
	builder.append("robot", robot).append("stop", true);
	robot_memory->insert(builder.obj(), "syncedrobmem.stopPlan");

	auto& robotPlan(Plan[robot]);
	for ( auto index = robotPlan.FirstNotDone; index < robotPlan.Tasks.size(); ++index )
	{
		removeFromPlanDB(robot, index);
	} //for ( auto index = robotPlan.FirstNotDone; index < robotPlan.Tasks.size(); ++index )
	robotPlan.Tasks.erase(robotPlan.Tasks.begin() + robotPlan.FirstNotDone, robotPlan.Tasks.end());
	return;
}

/**
 * @brief Gets called if there is feedback from one of the robots.
 * @param[in] document The document with the feedback.
 */
void
AspPlanerThread::planFeedbackCallback(const mongo::BSONObj document)
{
	try
	{
		const auto object(document.getField("o"));
		const auto action(object["action"].String());
		const auto robot(object["robot"].String());
		const auto task(taskCLIPStoASP(object["task"].String()));

		MutexLocker planLocker(&PlanMutex);
		auto& robotPlan(Plan[robot]);
		logger->log_info(LoggingComponent, "Plan-Feedback, R: %s T: %s A: %s CT: %s", robot.c_str(), task.c_str(),
			action.c_str(), robotPlan.CurrentTask.c_str());
		planLocker.unlock();

		//Switch only the first character because it is unique and we skip all the string handling.
		switch ( action[0] )
		{
			case 'b' :
			{
				robotBegunWithTask(robot, task, object["begin"].Long(), object["end"].Long());
				break;
			} //case 'b'
			case 'u' :
			{
				robotUpdatesTaskTimeEstimation(robot, task, object["end"].Long());
				break;
			} //case 'u'
			case 'e' :
			{
				robotFinishedTask(robot, task, object["end"].Long(), object["success"].String() == "TRUE");
				break;
			} //case 'e'
			default : throw fawkes::Exception("Unknown action %s!", action.c_str());
		} //switch ( action[0] )
	} //try
	catch ( const std::exception& e )
	{
		logger->log_error(LoggingComponent, "Exception while extracting plan feedback: %s\n%s", e.what(),
			document.toString().c_str());
	} //catch ( const std::exception& e )
	catch ( ... )
	{
		logger->log_error(LoggingComponent, "Exception while extracting plan feedback.\n%s",
			document.toString().c_str());
	} //catch ( ... )
	return;
}

