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

#include <chrono>
#include <unordered_map>
#include <type_traits>
#include <vector>

//! @todo Replace include and using, once C++17 is implemented properly.
#include <experimental/string_view>
using std::experimental::string_view;

class EventTrigger;

namespace fawkes {
	class MutexLocker;
}

using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;

enum class InterruptSolving : short
{
	Not = 0,
	JustStarted,
	Normal,
	High,
	Critical
};

struct BasicPlanElement
{
	std::string Task;
	int Begin = 0;
	int End = 0;
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
	bool Begun = false;
	bool Done = false;

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

	inline void
	updateTime(const BasicPlanElement& e) noexcept
	{
		if ( e.Begin != 0 )
		{
			Begin = e.Begin;
		} //if ( e.Begin != 0 )
		End = e.End;
		return;
	}

	inline void
	updateTimeAndTask(const BasicPlanElement& e)
		noexcept(noexcept(std::is_nothrow_copy_assignable<decltype(e.Task)>::value))
	{
		updateTime(e);
		Task = e.Task;
		return;
	}
};

inline bool operator==(const PlanElement& e1, const PlanElement& e2) noexcept
{
	return static_cast<BasicPlanElement>(e1) == static_cast<BasicPlanElement>(e2) && e1.Done == e2.Done &&
		e1.Begun == e2.Begun;
}

inline bool operator!=(const PlanElement& e1, const PlanElement& e2) noexcept
{
	return !(e1 == e2);
}

struct RobotPlan
{
	std::vector<PlanElement> Tasks;
	std::size_t FirstNotDone = 0;
	std::size_t FirstNotDoneOnSolveStart = 0;
	std::string CurrentTask;
};

struct ProductIdentifier
{
	int ID = -1;

	inline bool
	isValid(void) const noexcept
	{
		return ID != -1;
	}
};

struct Product
{
	std::string Base;
	// +1 to skip the calculation of +1 and -1 everytime we use this.
	std::string Rings[4];
	std::string Cap;
};

struct TaskDescription
{
	enum
	{
		None,
		Deliver,
		FeedRS,
		GetBase,
		GetProduct,
		Goto,
		MountCap,
		MountRing,
		PrepareCS
	} Type = None;
	Clingo::Symbol TaskSymbol;
	int EstimatedEnd = -1;

	inline bool
	isValid(void) const noexcept
	{
		return Type != None;
	}

	inline Clingo::Symbol
	location(void) const
	{
		assert(isValid());
		return TaskSymbol.arguments()[0];
	}
};

struct RobotInformation
{
	TimePoint LastSeen;
	bool Alive;
	Clingo::Symbol AliveExternal;
	float X;
	float Y;
	ProductIdentifier Holding;
	TaskDescription Doing;
};

struct MachineInformation
{
	int BrokenUntil = 0;
	int WorkingUntil = 0;
	ProductIdentifier Storing;
	int FillState = 0;
	bool Prepared = false;
	std::string State = "(not set)";
};

struct CapColorInformation
{
	std::string Color;
	std::string Machine;
};

struct RingColorInformation
{
	std::string Color;
	std::string Machine;
	int Cost;
};

struct OrderInformation
{
	int Number;
	int Quantity;
	std::string Base;
	std::string Cap;
	// +1 to skip the calculation of +1 and -1 everytime we use this.
	std::string Rings[4];
	int DeliveryBegin;
	int DeliveryEnd;
};

struct OrderTasks
{
	// +1 to skip the calculation of +1 and -1 everytime we use this.
	Clingo::Symbol RingTasks[4];
	Clingo::Symbol CapTask;
	Clingo::Symbol DeliverTasks[2];
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
inline Symbol
String(const std::string& str)
{
	return String(str.c_str());
}

}

#endif
