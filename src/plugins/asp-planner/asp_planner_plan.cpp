/***************************************************************************
 *  asp_planner_plan.cpp - ASP-based planner plugin plan composition
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

#include "asp_planner_thread.h"

#include <core/threading/mutex_locker.h>
#include <plugins/robot-memory/robot_memory.h>

#include <algorithm>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/document/value.hpp>
#include <cstdio>
#include <cstring>
#include <limits>
#include <type_traits>

using fawkes::MutexLocker;

/**
 * @brief Initalized plan elements.
 */
void
AspPlannerThread::initPlan(void)
{
	// Clear old plan, if in db.
	robot_memory->drop_collection("syncedrobmem.plan");
	return;
}

/**
 * @brief Used to sort plan elements by their beginning time.
 * @param[in] e1 The first element.
 * @param[in] e2 The second element.
 * @return If e1 begins before e2.
 */
static inline bool
planBegin(const BasicPlanElement &e1, const BasicPlanElement &e2) noexcept
{
	return e1.Begin < e2.Begin;
}

/**
 * @brief A intermediate representation of the plan elements.
 */
struct IntermediatePlanElement : public BasicPlanElement
{
	/** @brief The time to compare two Elements. */
	int *CompareTime;

	IntermediatePlanElement(void) = delete;

	/** Deleted dtor.
   * @param pe plan element
   */
	IntermediatePlanElement(const IntermediatePlanElement &pe) = delete;

	/**
   * @brief Move Constructor.
   * @param[in, out] that Moved element.
   */
	IntermediatePlanElement(IntermediatePlanElement &&that)
	: BasicPlanElement{std::move(that.Task), that.Begin, that.End},
	  CompareTime((that.CompareTime == &that.Begin) ? &Begin : &End)
	{
		return;
	}

	/**
   * @brief Constructor.
   * @param[in, out] task The task name.
   * @param[in] useBegin If the time to set is the begin time.
   * @param[in] time The time to set.
   */
	inline IntermediatePlanElement(std::string &&task, const bool useBegin, const int time)
	: BasicPlanElement{std::move(task)}, CompareTime(useBegin ? &Begin : &End)
	{
		*CompareTime = time;
		return;
	}

	/**
   * @brief Move Assignment.
   * @param [in, out] that Moved element.
   * @return This.
   */
	IntermediatePlanElement &
	operator=(IntermediatePlanElement &&that)
	{
		Task        = std::move(that.Task);
		Begin       = that.Begin;
		End         = that.End;
		CompareTime = (that.CompareTime == &that.Begin) ? &Begin : &End;
		return *this;
	}
};

/**
 * @brief Compares two IntermediatePlanElements for sorting.
 * @param[in] e1 The first element.
 * @param[in] e2 The second element.
 * @return If e1's time is smaller than e2's.
 */
inline bool
operator<(const IntermediatePlanElement &e1, const IntermediatePlanElement &e2)
{
	// If the times are equal we have one begin and one end, the end has to be
	// before that.
	return *(e1.CompareTime) < *(e2.CompareTime)
	       || (*(e1.CompareTime) == *(e2.CompareTime) && e1.End > e2.End);
}

/**
 * @brief Extracts the task information from the answer set and stores it in a
 * (robot) -> (task,begin,end) map, where only begin or end is set.
 * @param[in] symbols The answer set.
 * @param[out] map The map. The elements in the robot lists are sorted.
 * @param[in] planGameTime The game time the planning was started.
 * @param[in] transform Transforms the asp game time, to the real game time.
 * @param[in] log Wrapper around the fawkes logger.
 */
static inline void
extractMapFromAnswerSet(const Clingo::SymbolVector &symbols,
                        std::unordered_map<std::string, std::vector<IntermediatePlanElement>> &map,
                        const int                      planGameTime,
                        const std::function<int(int)> &transform,
                        fawkes::Logger *               logger,
                        const char *                   logging_component)
{
	for (auto &pair : map) {
		pair.second.clear();
	} // for ( auto& pair : map )

	for (const auto &symbol : symbols) {
		const string_view name(symbol.name());
		const bool        begin = name == "begin", end = name == "end";
		if (begin || end) {
			// Assumes begin(robot, task, time) and end(robot, task, time).
			const decltype(auto) args(symbol.arguments());
			const auto           time = args[2].number();
			const auto           robot(args[0].string());

			// If not in the map until now it will add a list for the robot.
			map[robot].emplace_back(args[1].to_string(),
			                        begin,
			                        std::max(transform(time) + planGameTime, 1));
		} // if ( begin || end )
	}   // for ( const auto& symbol : symbols )

	int size = 0;

	for (auto &pair : map) {
		std::sort(pair.second.begin(), pair.second.end());
		size += pair.second.size();
	} // for ( auto& pair : map )

	if (!size) {
		return;
	} // if ( !size )

	logger->log_info(logging_component, "Extracted plan size: %d Start GT: %d", size, planGameTime);
	return;
}

/**
 * @brief Takes the map from extractMapFromAnswerSet and assembles the tuples
 * (robot,task,begin,end) and stores them in sorted order in a vector for each
 * robot.
 * @param[in, out] map The map. Tasknames are moved out of the map.
 * @param[out] tempPlan The temporary plan resulting from the map.
 * @param[in] plan The currently deployed plan.
 * @param[in] log Wrapper around the fawkes logger.
 */
static inline void
assembleTemporaryPlan(std::unordered_map<std::string, std::vector<IntermediatePlanElement>> &map,
                      std::unordered_map<std::string, std::vector<BasicPlanElement>> &tempPlan,
                      const std::unordered_map<std::string, RobotPlan> &              plan,
                      fawkes::Logger *                                                logger,
                      const char *logging_component)
{
	for (auto &tempRobotPlan : tempPlan) {
		tempRobotPlan.second.clear();
	} // for ( auto& tempRobotPlan : tempPlan )

	std::size_t max = 0, min = std::numeric_limits<std::size_t>::max();

	for (auto &pair : map) {
		const auto &robotName(pair.first);
		const auto  robotNameCStr(robotName.c_str());
		auto &      robotTempPlan(tempPlan[robotName]);

		auto       iter = pair.second.begin();
		const auto end  = pair.second.end();

		if (iter != end && iter->Begin == 0) {
			// The first entry is a already started task, fix its begin time!
			const auto &robotPlan(plan.at(robotName));

			// But the task may already be done, search from the beginning of this
			// solve iteration.
			auto i = robotPlan.FirstNotDoneOnSolveStart;
			for (; i < robotPlan.Tasks.size(); ++i) {
				if (iter->Task == robotPlan.Tasks[i].Task) {
					break;
				} // if ( iter->Task == robotPlan.Tasks[i].Task )
			}   // for ( ; i < robotPlan.Tasks.size(); ++i )
			iter->Begin = robotPlan.Tasks[i].Begin;

			logger->log_info(logging_component,
			                 "Plan element: (%s, %s, %d, %d)",
			                 robotNameCStr,
			                 iter->Task.c_str(),
			                 iter->Begin,
			                 iter->End);
			robotTempPlan.emplace_back(std::move(*iter));
			++iter;
		} // if ( iter != end && iter->Begin == 0 )

		while (iter != end) {
			const auto next = iter + 1;
			assert(iter->End == 0);

			if (next == end) {
				/* The task has only a begin time, i.e. it is started shortly before our
         * lookahead ends. So the end time is out of the loohahead. */

				logger->log_info(logging_component,
				                 "Plan element: (%s, %s, %d, 0)",
				                 robotNameCStr,
				                 iter->Task.c_str(),
				                 iter->Begin);
				robotTempPlan.emplace_back(BasicPlanElement{std::move(iter->Task), iter->Begin, 0});
				break;
			} // if ( next == end )

			assert(iter->Task == next->Task);
			assert(next->Begin == 0);

			logger->log_info(logging_component,
			                 "Plan element: (%s, %s, %d, %d)",
			                 robotNameCStr,
			                 iter->Task.c_str(),
			                 iter->Begin,
			                 next->End);
			robotTempPlan.emplace_back(BasicPlanElement{std::move(iter->Task), iter->Begin, next->End});

			iter += 2;
		} // while ( iter != end )

		max = std::max(max, robotTempPlan.size());
		min = std::min(min, robotTempPlan.size());
	} // for ( auto& pair : map )

	if (max == 0) {
		logger->log_info(logging_component, "No tasks in plan.");
	} // if ( max == 0 )
	else {
		logger->log_info(logging_component, "Min elements per robot: %zu, max: %zu", min, max);
	} // else -> if ( max == 0 )
	return;
}

/**
 * @brief Handles everything concerning the plan in the loop.
 */
void
AspPlannerThread::loopPlan(void)
{
	MutexLocker solvingLocker(&SolvingMutex);
	if (!NewSymbols) {
		MutexLocker worldLocker(&WorldMutex);
		return;
	} // if ( !NewSymbols )

	static const std::chrono::milliseconds planThreshold(
	  config->get_int("/asp-agent/planner/plan-extraction-delay"));
	if (Clock::now() - LastModel < planThreshold) {
		// The last model is fresh, maybe there is a new and better in the next
		// loop, wait a short time.
		return;
	} // if ( Clock::now() - LastModel < planThreshold )

	if (LastModel < SolvingStarted) {
		// The last model is based on old information, do not use it! Without this
		// there were wrong plans!
		return;
	} // if ( LastModel < SolvingStarted )

	/* There is a new model and it is old enough, start with the plan extraction.
   * Since we have no guarantees about the ordering in the model we have to
   * create the order ourselves. */
	MutexLocker planLocker(&PlanMutex);

	// Make this map static, so we do not release the needed memory everytime.
	static std::unordered_map<std::string, std::vector<IntermediatePlanElement>> map;

	PlanGameTime = StartSolvingGameTime;
	extractMapFromAnswerSet(
	  Symbols,
	  map,
	  PlanGameTime,
	  [this](int gt) noexcept { return aspGameTimeToRealGameTime(gt); },
	  logger,
	  LoggingComponent);
	NewSymbols = false;
	solvingLocker.unlock();
	LastPlan = Clock::now();

	// Assemble a plan only based on the information from clingo.
	static std::unordered_map<std::string, std::vector<BasicPlanElement>> tempPlan;

	assembleTemporaryPlan(map, tempPlan, Plan, logger, LoggingComponent);

	// Some helper functions to handle plan elements.
	// If two plan elements refer to the same task.
	auto sameTask = [](const BasicPlanElement &e1, const BasicPlanElement &e2) noexcept
	{
		return e1.Task == e2.Task;
	};
	// Compares the begin and end time point and if their difference is above a
	// given threshold. This is only valid if both elements are about the same
	// task.
	auto needsUpdate =
	  [this, sameTask](const BasicPlanElement &e1, const BasicPlanElement &e2) noexcept
	{
		assert(sameTask(e1, e2));
		static_assert(std::is_signed<decltype(e1.Begin)>::value,
		              "For unsigned values std::abs doesn't make any sense.");
		const auto threshold = TimeResolution / 2;
		if (e2.Begin == 0) {
			/* This is a task, which was running when the solving started, since we
       * never have a begin() the value is not set. So just compare the end
       * values. */
			return std::abs(e1.End - e2.End) > threshold;
		} // if ( e2.Begin == 0 )
		return std::abs(e1.Begin - e2.Begin) > threshold || std::abs(e1.End - e2.End) > threshold;
	};

	/* We copy the deployed plan into planCopy and perform changes on it, if we do
   * not encounter a problem with the new plan from tempPlan we commit the
   * changes, i.e. swap it over to the Plan and update the database. */
	bool commit = true;
	// Static as usual to hold onto the allocated memory.
	static std::map<decltype(Plan)::key_type, decltype(Plan[""].Tasks)> planCopy;

	for (const auto &pair : Plan) {
		planCopy[pair.first] = pair.second.Tasks;
	} // for ( const auto& pair : Plan )

	for (auto &pair : tempPlan) {
		const auto &robotName(pair.first);
		auto &      robotPlan(planCopy[robotName]);
		auto &      tempRobotPlan(pair.second);

		logger->log_info(LoggingComponent,
		                 "Update plan for %s, %zu elements.",
		                 robotName.c_str(),
		                 tempRobotPlan.size());

		// Get the index from the first task regarding to this solve iteration.
		int index = Plan[robotName].FirstNotDoneOnSolveStart;
		// Get from that the iterator.
		auto planIter = robotPlan.begin() + index;
		// Cache the end iterator of the deployed robot plan.
		const auto planEnd = robotPlan.end();

		// The two iterators of the temp plan.
		auto       tempIter = tempRobotPlan.begin();
		const auto tempEnd  = tempRobotPlan.end();

		if (planIter == planEnd) {
			logger->log_info(LoggingComponent,
			                 "Start at index %d, do not change the existing plan.",
			                 index);
		} // if ( planIter == planEnd )
		else {
			logger->log_info(LoggingComponent,
			                 "Start at index %d, first existing entry to compare is: "
			                 "(%s,%s,%d,%d)",
			                 index,
			                 robotName.c_str(),
			                 planIter->Task.c_str(),
			                 planIter->Begin,
			                 planIter->End);
		} // else -> if ( planIter == planEnd )

		// There is an entry in our old plan, which competes with an entry of the
		// new plan for the same index.
		while (planIter != planEnd && tempIter != tempEnd) {
			if (sameTask(*planIter, *tempIter)) {
				// Everything is fine, we have the same task, maybe only update the
				// timing.
				if (needsUpdate(*planIter, *tempIter) && !planIter->Done) {
					logger->log_info(LoggingComponent,
					                 "Update time for (%s,%s,%d,%d) to (%d,%d)",
					                 robotName.c_str(),
					                 planIter->Task.c_str(),
					                 planIter->Begin,
					                 planIter->End,
					                 tempIter->Begin,
					                 tempIter->End);
					if (planIter->Begun) {
						logger->log_warn(LoggingComponent, "The task is already started!");
						/* Do not change the begin time for a already begun task, this
             * breaks the evaluation based on the plan. */
						tempIter->Begin = planIter->Begin;
					} // if ( planIter->Begun )
					planIter->updateTime(*tempIter);
				} // if ( needsUpdate(*planIter, *tempIter) && !planIter->Done )
				else {
					logger->log_info(LoggingComponent,
					                 "Leave (%s,%s,%d,%d) unchanged.",
					                 robotName.c_str(),
					                 planIter->Task.c_str(),
					                 planIter->Begin,
					                 planIter->End);
				} // else -> if ( needsUpdate(*planIter, *tempIter) && !planIter->Done )
			}   // if ( sameTask(*planIter, *tempIter) )
			else {
				// A different task, handle differently for already started tasks.
				const auto robotCStr = robotName.c_str();
				if (planIter->Begun) {
					logger->log_warn(LoggingComponent,
					                 "Should change started task %s from robot %s to %s. "
					                 "Restart solving!",
					                 planIter->Task.c_str(),
					                 robotCStr,
					                 tempIter->Task.c_str());

					commit = false;
					break;
				} // if ( planIter->Begun )

				logger->log_info(LoggingComponent,
				                 "Change from (%s,%s,%d,%d) to (%s,%s,%d,%d).",
				                 robotCStr,
				                 planIter->Task.c_str(),
				                 planIter->Begin,
				                 planIter->End,
				                 robotCStr,
				                 tempIter->Task.c_str(),
				                 tempIter->Begin,
				                 tempIter->End);
				planIter->updateTimeAndTask(*tempIter);
			} // else -> if ( sameTask(*planIter, *tempIter) )
			++planIter;
			++tempIter;
			++index;
		} // while ( planIter != planEnd && tempIter != tempEnd )

		if (!commit) {
			// Stop right here.
			break;
		} // if ( !commit )

		if (tempIter != tempEnd) {
			// We have more tasks in our temp plan than in the deployed plan left.
			robotPlan.reserve(robotPlan.size() + tempEnd - tempIter);
			std::copy(tempIter, tempEnd, std::back_inserter(robotPlan));
			do // while ( ++index < static_cast<int>(robotPlan.size()) )
			{
				const auto &entry(robotPlan[index]);
				logger->log_info(LoggingComponent,
				                 "Add task (%s,%s,%d,%d) at index %d.",
				                 robotName.c_str(),
				                 entry.Task.c_str(),
				                 entry.Begin,
				                 entry.End,
				                 index);
			} while (++index < static_cast<int>(robotPlan.size()));
		} // if ( tempIter != tempEnd )
		else if (planIter != planEnd) {
			// We have less tasks in our temp plan than in the deployed plan left.
			if (planIter->Begun) {
				// Deleting a task which is already started seems not to be the best
				// idea, restart solving.
				logger->log_warn(LoggingComponent,
				                 "Should delete started task %s for robot %s! Restart solving.",
				                 planIter->Task.c_str(),
				                 robotName.c_str());
				commit = false;
				break;
			} // if ( planIter->Begun )

			logger->log_info(LoggingComponent,
			                 "Remove %zu elements from (%s,%s,%d,%d) on.",
			                 planEnd - planIter,
			                 robotName.c_str(),
			                 planIter->Task.c_str(),
			                 planIter->Begin,
			                 planIter->End);
			robotPlan.erase(planIter, planEnd);
		} // else if ( planIter != planEnd )
	}   // for ( auto& pair : Plan )

	if (commit) {
		logger->log_info(LoggingComponent,
		                 "Plan updating done, commit changes into the deployed plan.");

		// To commit the changes we only have to update the database and make a swap
		// at the end of the loop.
		for (const auto &pair : Robots) {
			const auto &robot(pair.first);
			auto &      deployed(Plan[robot].Tasks);
			auto &      updated(planCopy[robot]);

			int       index = 0;
			const int dSize(deployed.size()), uSize(updated.size());
			for (; index < dSize && index < uSize; ++index) {
				if (sameTask(deployed[index], updated[index])) {
					if (needsUpdate(deployed[index], updated[index])) {
						updatePlanTiming(robot, index, updated[index]);
					} // if ( needsUpdate(deployed[index], updated[index]) )
				}   // if ( sameTask(deployed[index], updated[index]) )
				else {
					updatePlan(robot, index, updated[index]);
				} // else -> if ( sameTask(deployed[index], updated[index]) )
			}   // for ( ; index < dSize && index < uSize; ++index )

			// At most one of the following loops will be executed.
			for (; index < dSize; ++index) {
				removeFromPlanDB(robot, index);
			} // for ( ; index < dSize; ++index )

			for (; index < uSize; ++index) {
				insertPlanElement(robot, index, updated[index]);
			} // for ( ; index < uSize; ++index )

			using std::swap;
			swap(deployed, updated);
		} // for ( const auto& pair : Robots )
	}   // if ( commit )
	else {
		logger->log_info(LoggingComponent,
		                 "The new plan will be discarded because of inconsistencies.");
		setInterrupt(InterruptSolving::Critical, "Inconsistent plan");
	} // else -> if ( commit )
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
	std::transform(
	  string.begin(), string.end(), string.begin(), [](const char c) noexcept {
		  switch (c) {
		  case ',': return ' ';
		  case '"': return '\'';
		  } // switch ( c )
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
	std::transform(
	  string.begin(), string.end(), string.begin(), [](const char c) noexcept {
		  switch (c) {
		  case ' ': return ',';
		  case '\'': return '"';
		  } // switch ( c )
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
static bsoncxx::document::value
createQuery(const std::string &robot, const int elementIndex)
{
	using namespace bsoncxx::builder;
	return basic::make_document(basic::kvp("relation", "planElement"),
	                            basic::kvp("robot", robot),
	                            basic::kvp("index", elementIndex));
}

/**
 * @brief Heper function to create a BSON object out of a PlanElement.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element, used for the ordering on
 * the executive. Is ignored if set to -1.
 * @param[in] element The element.
 * @return The object.
 */
static bsoncxx::document::value
createObject(const std::string &robot, const int elementIndex, const PlanElement &element)
{
	using namespace bsoncxx::builder;
	basic::document doc;
	doc.append(basic::kvp("relation", "planElement"));
	doc.append(basic::kvp("robot", robot));
	doc.append(basic::kvp("task", "\"" + taskASPtoCLIPS(element.Task) + "\""));
	doc.append(basic::kvp("begin", element.Begin));
	doc.append(basic::kvp("end", element.End));
	if (elementIndex >= 0) {
		doc.append(basic::kvp("index", elementIndex));
	} // if ( elementIndex >= 0 )
	return doc.extract();
}

/**
 * @brief Inserts a new element in the plan.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element, used for the ordering on
 * the executive.
 * @param[in] element The element.
 */
void
AspPlannerThread::insertPlanElement(const std::string &robot,
                                    const int          elementIndex,
                                    const PlanElement &element)
{
	robot_memory->insert(createObject(robot, elementIndex, element), "syncedrobmem.plan");
	return;
}

/**
 * @brief Updates the plan element, actually replaces it.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element, used for the ordering on
 * the executive.
 * @param[in] element The element.
 */
void
AspPlannerThread::updatePlan(const std::string &robot,
                             const int          elementIndex,
                             const PlanElement &element)
{
	robot_memory->update(createQuery(robot, elementIndex),
	                     createObject(robot, elementIndex, element),
	                     "syncedrobmem.plan",
	                     true);
	return;
}

/**
 * @brief Updates the plan elements time in the robot memory.
 * @param[in] robot For which robot the plan element is.
 * @param[in] elementIndex The index of the element, used for the ordering on
 * the executive.
 * @param[in] element The element.
 */
void
AspPlannerThread::updatePlanTiming(const std::string &robot,
                                   const int          elementIndex,
                                   const PlanElement &element)
{
	/* The idea was only to update the time, but this either don't work, or my
   * query wasn't good enough, so call the general update method. */
	updatePlan(robot, elementIndex, element);
	return;
}

/**
 * @brief Removes a plan element from the robot memory.
 * @param[in] robot For which robot the plan element is.
 * @param[in] element The element.
 */
void
AspPlannerThread::removeFromPlanDB(const std::string &robot, const int elementIndex)
{
	robot_memory->remove(createQuery(robot, elementIndex), "syncedrobmem.plan");
	return;
}

/**
 * @brief Tells the robot to stop its current task and removes its plan, because
 * the plan is corrupted.
 * @param[in] robot The robots name.
 * @note The plan lock has to be hold.
 */
void
AspPlannerThread::tellRobotToStop(const std::string &robot)
{
	using namespace bsoncxx::builder;
	basic::document doc;
	doc.append(basic::kvp("robot", robot));
	doc.append(basic::kvp("stop", true));
	robot_memory->insert(doc.extract(), "syncedrobmem.stopPlan");

	auto &robotPlan(Plan[robot]);
	for (auto index = robotPlan.FirstNotDone; index < robotPlan.Tasks.size(); ++index) {
		removeFromPlanDB(robot, index);
	} // for ( auto index = robotPlan.FirstNotDone; index <
	// robotPlan.Tasks.size(); ++index )
	robotPlan.Tasks.erase(robotPlan.Tasks.begin() + robotPlan.FirstNotDone, robotPlan.Tasks.end());
	return;
}

/**
 * @brief Gets called if there is feedback from one of the robots.
 * @param[in] document The document with the feedback.
 */
void
AspPlannerThread::planFeedbackCallback(const bsoncxx::document::view &document)
{
	try {
		const auto object(document["o"]);
		const auto action(object["action"].get_utf8().value.to_string());
		const auto robot(object["robot"].get_utf8().value.to_string());
		const auto task(taskCLIPStoASP(object["task"].get_utf8().value.to_string()));

		MutexLocker planLocker(&PlanMutex);
		auto &      robotPlan(Plan[robot]);
		logger->log_info(LoggingComponent,
		                 "Plan-Feedback, R: %s T: %s A: %s CT: %s",
		                 robot.c_str(),
		                 task.c_str(),
		                 action.c_str(),
		                 robotPlan.CurrentTask.c_str());
		planLocker.unlock();

		// Switch only the first character because it is unique and we skip all the
		// string handling.
		switch (action[0]) {
		case 'b': {
			robotBegunWithTask(robot, task, object["begin"].get_int64(), object["end"].get_int64());
			break;
		} // case 'b'
		case 'u': {
			robotUpdatesTaskTimeEstimation(robot, task, object["end"].get_int64());
			break;
		} // case 'u'
		case 'e': {
			robotFinishedTask(robot,
			                  task,
			                  object["end"].get_int64(),
			                  object["success"].get_utf8().value.to_string() == "TRUE");
			break;
		} // case 'e'
		default: throw fawkes::Exception("Unknown action %s!", action.c_str());
		} // switch ( action[0] )
	}   // try
	catch (const std::exception &e) {
		logger->log_error(LoggingComponent,
		                  "Exception while extracting plan feedback: %s\n%s",
		                  e.what(),
		                  bsoncxx::to_json(document).c_str());
	} // catch ( const std::exception& e )
	catch (...) {
		logger->log_error(LoggingComponent,
		                  "Exception while extracting plan feedback.\n%s",
		                  bsoncxx::to_json(document).c_str());
	} // catch ( ... )
	return;
}
