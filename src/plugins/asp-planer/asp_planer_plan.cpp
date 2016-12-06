/***************************************************************************
 *  asp_planer_plan.cpp - ASP-based planer plugin plan composition
 *
 *  Created on Tue Aug 23 13:57:02 2016
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

#include <cstring>

#include <core/threading/mutex_locker.h>

using fawkes::MutexLocker;

/**
 * @property AspPlanerThread::LastPlan
 * @brief When the last plan was composed.
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

		std::unordered_map<std::pair<std::string, std::string>, std::pair<unsigned int, unsigned int>> map;
		for ( const auto& symbol : Symbols )
		{
			const bool begin = std::strcmp(symbol.name(), "begin") == 0,
				end = std::strcmp(symbol.name(), "end") == 0;
			if ( begin || end )
			{
				const auto args(symbol.arguments());
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

		logger->log_info(LoggingComponent, "Plan size: %d", map.size());
		for ( const auto& pair : map )
		{
			logger->log_info(LoggingComponent, "Plan element: (%s, %s, %d, %d)",
				pair.first.first.c_str(), pair.first.second.c_str(), pair.second.first, pair.second.second);
		} //for ( const auto& pair : map )
		NewSymbols = false;
		LastPlan = clock->now();
	} //if ( NewSymbols )
	return;
}
