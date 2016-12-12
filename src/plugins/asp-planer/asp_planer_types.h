/***************************************************************************
 *  asp_planer_types.h - ASP-based planer plugin
 *
 *  Header for extra types, not in the thread.h for a better overview.
 *
 *  Created on Thu Dec 08 12:01:02 2016
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

#ifndef __PLUGINS_ASP_AGENT_TYPES_H_
#define __PLUGINS_ASP_AGENT_TYPES_H_

#include <clingo.hh>

#include <unordered_map>
#include <vector>

#include <libs/utils/time/time.h>

class EventTrigger;

namespace fawkes {
	class MutexLocker;
}

enum class InterruptSolving : unsigned short
{
	Not = 0,
	JustStarted,
	Normal,
	Critical
};

struct GroundRequest
{
	const char *Name;
	Clingo::SymbolVector Params;
	bool AddTick;
};

struct BasicPlanElement
{
	std::string Task;
	unsigned int Begin;
	unsigned int End;
};

inline bool operator==(const BasicPlanElement& e1, const BasicPlanElement& e2) noexcept
{
	return e1.Begin == e2.Begin && e1.End == e2.End && e1.Task == e2.Task;
}

inline bool operator!=(const BasicPlanElement& e1, const BasicPlanElement& e2) noexcept
{
	return !(e1 == e2);
}

struct PlanElement : public BasicPlanElement
{
	bool Done = false;

	bool Visited = false;
	enum {
		Nothing,
		Insert,
		Update,
	} Action = Nothing;

	//Construct from base class.
	inline PlanElement(const BasicPlanElement& b) noexcept : BasicPlanElement(b)
	{
		return;
	}
	inline PlanElement(BasicPlanElement&& b) noexcept : BasicPlanElement(std::move(b))
	{
		return;
	}

	//Reactivate default copy constructors and assignment operators.
	inline PlanElement(const PlanElement& e) = default;
	inline PlanElement(PlanElement&& e) noexcept = default;
	inline PlanElement& operator=(const PlanElement& e) = default;
	inline PlanElement& operator=(PlanElement&& e) noexcept = default;
};

inline bool operator==(const PlanElement& e1, const PlanElement& e2) noexcept
{
	return static_cast<BasicPlanElement>(e1) == static_cast<BasicPlanElement>(e2) && e1.Done == e2.Done &&
		e1.Visited == e2.Visited && e1.Action == e2.Action;
}

inline bool operator!=(const PlanElement& e1, const PlanElement& e2) noexcept
{
	return !(e1 == e2);
}

struct RobotPlan
{
	std::vector<PlanElement> Plan;
	std::size_t FirstNotDone = 0;
	std::string CurrentTask;
};

struct RobotInformation
{
	fawkes::Time LastSeen;
	float X;
	float Y;
	unsigned int GameTime;
};

struct RingColorInformation
{
	std::string Color;
	std::string Machine;
	unsigned int Cost;
};

struct OrderInformation
{
	unsigned int Number;
	unsigned int Quantity;
	std::string Base;
	std::string Cap;
	std::string Rings[3];
	unsigned int DeliveryBegin;
	unsigned int DeliveryEnd;
	unsigned int GameTime;
};

namespace std {

/**
 * @brief Helper class to instantiate a std::unordered_map with a std::pair as key.
 */
template<typename T1, typename T2>
struct hash<pair<T1, T2>>
{
	auto operator()(const pair<T1, T2>& pair) const
		noexcept(noexcept(hash<T1>{}(pair.first) && noexcept(hash<T2>{}(pair.second))))
	{
		//Is this a good hash?
		return hash<T1>{}(pair.first) << 16 ^ hash<T2>{}(pair.second);
	}
};
}

namespace Clingo {
/**
 * @brief Simple convenience function.
 * @param[in] string The C++-String.
 * @return The Clingo::Symbol constructed from the string.
 */
inline Symbol String(const std::string& str)
{
	return String(str.c_str());
}
}

#endif
