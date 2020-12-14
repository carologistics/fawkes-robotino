/***************************************************************************
 *  asp_planner_types.h - ASP-based planner plugin
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

#include <chrono>
#include <clingo.hh>
#include <type_traits>
#include <unordered_map>
#include <vector>

//! @todo Replace include and using, once C++17 is implemented properly.
#include <experimental/string_view>
using std::experimental::string_view;

class EventTrigger;

namespace fawkes {
class MutexLocker;
}

using Clock     = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;

enum class InterruptSolving : short { Not = 0, JustStarted, Normal, High, Critical };

inline constexpr const char *
interruptString(const InterruptSolving interrupt)
{
	switch (interrupt) {
	case InterruptSolving::Not: return "Not";
	case InterruptSolving::JustStarted: return "JustStarted";
	case InterruptSolving::Normal: return "Normal";
	case InterruptSolving::High: return "High";
	case InterruptSolving::Critical: return "Critical";
	} // switch ( interrupt )
	return "";
}

struct BasicPlanElement
{
	std::string Task;
	int         Begin = 0;
	int         End   = 0;
};

inline bool
operator==(const BasicPlanElement &e1, const BasicPlanElement &e2) noexcept
{
	return e1.Begin == e2.Begin && e1.End == e2.End && e1.Task == e2.Task;
}

inline bool
operator!=(const BasicPlanElement &e1, const BasicPlanElement &e2) noexcept
{
	return !(e1 == e2);
}

struct PlanElement : public BasicPlanElement
{
	bool Begun  = false;
	bool Done   = false;
	bool Failed = false;

	/** Construct from base class.
   * @param b basic plan element
   */
	inline PlanElement(const BasicPlanElement &b) noexcept : BasicPlanElement(b)
	{
		return;
	}

	/** Construct from base class.
   * @param b basic plan element
   */
	inline PlanElement(BasicPlanElement &&b) noexcept : BasicPlanElement(std::move(b))
	{
		return;
	}

	/** Copy ctor.
   * @param e plan element to copy
   */
	inline PlanElement(const PlanElement &e) = default;
	/** Move ctor.
   * @param e plan element to move
   */
	inline PlanElement(PlanElement &&e) noexcept = default;
	/** Copying assignment operator.
   * @param e plan element to copy from
   * @return reference to this instance
   */
	inline PlanElement &operator=(const PlanElement &e) = default;
	/** Moving assignment operator.
   * @param e plan element to move
   * @return reference to this instance
   */
	inline PlanElement &operator=(PlanElement &&e) noexcept = default;

	/** Update time to that of the given element.
   * @param e plan element to copy time from
   */
	inline void
	updateTime(const BasicPlanElement &e) noexcept
	{
		if (e.Begin != 0) {
			Begin = e.Begin;
		} // if ( e.Begin != 0 )
		End = e.End;
		return;
	}

	/** Update time and task to that of another element.
   * @param e plan element to copy from
   */
	inline void
	updateTimeAndTask(const BasicPlanElement &e) noexcept(
	  noexcept(std::is_nothrow_copy_assignable<decltype(e.Task)>::value))
	{
		updateTime(e);
		Task = e.Task;
		return;
	}
};

/** Check equality of two plan elements.
 * @param e1 first plan element
 * @param e2 second plan element
 * @return true if plan elements are the same, false otherwise
 */
inline bool
operator==(const PlanElement &e1, const PlanElement &e2) noexcept
{
	return static_cast<BasicPlanElement>(e1) == static_cast<BasicPlanElement>(e2)
	       && e1.Done == e2.Done && e1.Begun == e2.Begun;
}

/** Check inequality of two plan elements.
 * @param e1 first plan element
 * @param e2 second plan element
 * @return true if plan elements are not the same, false otherwise
 */
inline bool
operator!=(const PlanElement &e1, const PlanElement &e2) noexcept
{
	return !(e1 == e2);
}

struct RobotPlan
{
	std::vector<PlanElement> Tasks;
	std::size_t              FirstNotDone             = 0;
	std::size_t              FirstNotDoneOnSolveStart = 0;
	std::string              CurrentTask;
};

struct ProductIdentifier
{
	int ID = -1;

	/** Check if ID is valid.
   * @return true if ID is valid, i.e. it is not -1.
   */
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
	/** Possible types. */
	enum {
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
	int            EstimatedEnd = -1;

	/** Check if type is valid.
   * @return true if type is not None */
	inline bool
	isValid(void) const noexcept
	{
		return Type != None;
	}

	/** Get location for task.
   * @return symbol representing location.
   **/
	inline Clingo::Symbol
	location(void) const
	{
		assert(isValid());
		return TaskSymbol.arguments()[0];
	}
};

struct RobotInformation
{
	TimePoint         LastSeen;
	bool              Alive;
	Clingo::Symbol    AliveExternal;
	float             X;
	float             Y;
	ProductIdentifier Holding;
	TaskDescription   Doing;
};

struct MachineInformation
{
	int               BrokenUntil  = 0;
	int               WorkingUntil = 0;
	ProductIdentifier Storing;
	int               FillState = 0;
	bool              Prepared  = false;
	std::string       State     = "(not set)";
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
	int         Cost;
};

struct OrderInformation
{
	int         Number;
	int         Quantity;
	std::string Base;
	std::string Cap;
	// +1 to skip the calculation of +1 and -1 everytime we use this.
	std::string Rings[4];
	int         DeliveryBegin;
	int         DeliveryEnd;
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
 * @brief Helper class to instantiate a std::unordered_map with a std::pair as
 * key.
 */
template <typename T1, typename T2>
struct hash<pair<T1, T2>>
{
	/** Calculate hash.
   * @param pair elements to create hash for
   * @return hash value
   */
	auto
	operator()(const pair<T1, T2> &pair) const
	  noexcept(noexcept(hash<T1>{}(pair.first) &&noexcept(hash<T2>{}(pair.second))))
	{
		// Is this a good hash?
		return hash<T1>{}(pair.first) << 16 ^ hash<T2>{}(pair.second);
	}
};
} // namespace std

namespace Clingo {
/**
 * @brief Simple convenience function.
 * @param[in] string The C++-String.
 * @return The Clingo::Symbol constructed from the string.
 */
inline Symbol
String(const std::string &str)
{
	return String(str.c_str());
}

} // namespace Clingo

#endif
