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
#include <cstring>

#include <core/threading/mutex_locker.h>

using fawkes::MutexLocker;

/**
 * @struct BasicPlanElement
 * @brief The basis of an element for the plan.
 *
 * @property BasicPlanElement::Task
 * @brief The task with it's parameters.
 *
 * @property BasicPlanElement::Begin
 * @brief The game time on which the task should start.
 *
 * @property BasicPlanElement::End
 * @brief The estimated end time for the task, if available. If not it is set to zero.
 */

/**
 * @struct PlanElement
 * @brief The saved plan element.
 *
 * @property PlanElement::Done
 * @brief Wether the task is done or not.
 *
 * @property PlanElement::Visited
 * @brief Wether this element was visited in the current sweep.
 *
 * @property PlanElement::Action
 * @brief What to do with this element in the syncronized plan.
 */

/**
 * @struct RobotPlan
 * @brief The plan for a robot.
 *
 * @property RobotPlan::Plan
 * @brief The list of PlanElements forming the plan.
 *
 * @property RobotPlan::FirstNotDone
 * @brief The index in the plan for the first task which is not done.
 */

/**
 * @property AspPlanerThread::PlanMutex
 * @brief The mutex for plan handling
 *
 * @property AspPlanerThread::LastPlan
 * @brief When the last plan was composed.
 *
 * @property AspPlanerThread::Plan
 * @brief The mapping from robot name to its plan.
 */

/**
 * @brief Handles everything concerning the plan in the loop.
 */
void
AspPlanerThread::loopPlan(void)
{
	MutexLocker symbolLocker(&SymbolMutex);
	if ( NewSymbols )
	{
		static const fawkes::Time planThreshold(500L);
		if ( clock->now() - LastModel < planThreshold )
		{
			//The last model is fresh, maybe there is a new and better in the next loop, wait a short time.
			return;
		} //if ( clock->now() - LastModel < planThreshold )

		/* Extract the information. Since we have no guarantees about the ordering in the model we use a map to assemble
		 * the (robot, task, begin, end) tuples. */
		std::unordered_map<std::pair<std::string, std::string>, std::pair<unsigned int, unsigned int>> map;
		//Reserve enough space for the known plan + 1 entry per robot.
		map.reserve(PlanElements + Robots.size());
		for ( const auto& symbol : Symbols )
		{
			const bool begin = std::strcmp(symbol.name(), "begin") == 0,
				end = std::strcmp(symbol.name(), "end") == 0;
			if ( begin || end )
			{
				//Assumes begin(robot, task, time) and end(robot, task, time).
				const decltype(auto) args(symbol.arguments());
				//If not in the map until now it will add (robot, task, 0, 0).
				auto& pair = map[{args[0].to_string(), args[1].to_string()}];
				if ( begin )
				{
					pair.first = args[2].number();
				} //if ( begin )
				else
				{
					pair.second = args[2].number();
				} //else -> if ( begin )
			} //if ( begin || end )
		} //for ( const auto& symbol : Symbols )

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
		//If two plan elements are considered equal, only call if sameTask already holds.
		//Compared are the begin and end time point.
		auto equal = [sameTask](const BasicPlanElement& e1, const BasicPlanElement& e2) noexcept
			{
				assert(sameTask(e1, e2));
				return e1.Begin == e2.Begin && e1.End == e2.End;
			};

		MutexLocker planLocker(&PlanMutex);
		//Prepare the plans.
//		for ( auto& plan : Plan )
//		{
//			const auto end = plan.second.Plan.end();
//			for ( auto iter = plan.second.Plan.begin() + plan.second.FirstNotDone; iter != end; ++iter )
//			{
//				iter->Visited = false;
//			} //for ( auto iter = plan.second.Plan.begin() + plan.second.FirstNotDone; iter != end; ++iter )
//		} //for ( auto& plan : Plan )

		logger->log_info(LoggingComponent, "New plan size: %d", map.size());
		for ( const auto& pair : map )
		{
			logger->log_info(LoggingComponent, "Plan element: (%s, %s, %d, %d)",
				pair.first.first.c_str(), pair.first.second.c_str(), pair.second.first, pair.second.second);

			//Get the correct robot plan.
			const auto& robot(pair.first.first);
			const BasicPlanElement element{pair.first.second, pair.second.first, pair.second.second};
			auto& robotPlan(Plan[robot]);
			auto& plan(robotPlan.Plan);

			//Search for the position in the plan of the element.
			auto iter = std::lower_bound(plan.begin(), plan.end(), element, planBegin);

			if ( sameTask(element, *iter) )
			{
				iter->Visited = true;
				if ( !equal(element, *iter) )
				{
					/* We have the same task, but begin and/or end differs, but the element is still sorted right in the
					 * plan. So we only update the element. */
					iter->Begin  = element.Begin;
					iter->End    = element.End;
					iter->Action = PlanElement::Update;
				} //if ( !equal(element, *iter) )
				//In the else case nothing is to do.
			} //if ( sameTask(element, *iter) )
			else
			{
				//We have a differnt task, so we add the element.
				iter = plan.emplace(iter, element);
				iter->Visited = true;
				iter->Action  = PlanElement::Insert;
			} //else -> if ( sameTask(element, *iter) )
		} //for ( const auto& pair : map )

		//Now sweep the plan for changes.
		for ( auto& pair : Plan )
		{
			const auto& robot(pair.first);
			auto& robotPlan(pair.second);
			auto& plan(robotPlan.Plan);

			//Skip the elements we consider as done.
			auto index = robotPlan.FirstNotDone;
			auto iter = plan.begin() + index, end = plan.end();
			bool newOrdering = false;

			while ( iter != end )
			{
				if ( iter->Visited )
				{
					//Handle element.
					switch ( iter->Action )
					{
						case PlanElement::Nothing :
						{
							/* We have to update the DB entry if we have a new ordering, when something before this
							 * index was added or removed. So only break when newOrderung is not set. */
							if ( !newOrdering )
							{
								break;
							} //if ( !newOrdering )
#if __has_cpp_attribute(fallthrough)
							//GCC 7, Clan 3.9 or later.
							[[fallthrough]];
#endif
						} //case PlanElement::Nothing
						case PlanElement::Update : updatePlanDB(robot, index, *iter); break;
						case PlanElement::Insert : newOrdering = true; updatePlanDB(robot, index, *iter); break;
					} //switch ( iter->Action )

					//Reset element for the next run.
					iter->Visited = false;
					iter->Action  = PlanElement::Nothing;

					//Go to the next element.
					++iter;
					++index;
				} //if ( iter->Visited )
				else
				{
					//The element was not visited in the plan extraction and is not done yet, so we remove it.
					removeFromPlanDB(robot, *iter);

					/* Remove also from the vector, but do not really remove, we use the erase-remove-idiom:
					 * The element to remove is moved to the end of the range by shifting, this preserves the ordering.
					 * iter will now point to the next element.
					 * Std::remove gives the iterator to the first element to erase, in our case the element we wan't to
					 * remove, so let end point to that. */
					end = std::remove(iter, end, *iter);
					newOrdering = true;
				} //else -> if ( iter->Visited )
			} //while ( iter != end )

			//Now erase the elements moved to the end of the vector.
			plan.erase(end, plan.end());
		} //for ( auto& pair : Plan )

		NewSymbols = false;
		LastPlan = clock->now();
	} //if ( NewSymbols )
	return;
}
