
/***************************************************************************
 *  asp_planer_thread.h - ASP-based planer plugin
 *
 *  Created on Thu Aug 18 04:20:02 2016
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

#ifndef __PLUGINS_ASP_AGENT_THREAD_H_
#define __PLUGINS_ASP_AGENT_THREAD_H_

#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>
#include <plugins/asp/aspect/asp.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>

#include <clingo.hh>

#include <unordered_map>
#include <vector>

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
};

struct RobotInformation
{
	fawkes::Time LastSeen;
	float X;
	float Y;
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

class AspPlanerThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ASPAspect,
  public fawkes::RobotMemoryAspect,
  public fawkes::NavGraphAspect,
  public fawkes::NavGraph::ChangeListener
{
	private:
	const char* const LoggingComponent;
	const char* const ConfigPrefix;
	std::vector<EventTrigger*> RobotMemoryCallbacks;
	const char* TeamColor;

	bool MoreModels;
	unsigned int ExplorationTime;

	unsigned int LookAhaed;
	unsigned int LastTick;
	unsigned int GameTime;
	unsigned int Horizon;
	unsigned int Past;
	fawkes::Time SolvingStarted;
	fawkes::Time LastModel;
	unsigned int MachinesFound;
	bool StillNeedExploring;
	bool CompleteRestart;
	unsigned int TimeResolution;
	unsigned int MaxDriveDuration;
	std::vector<std::string> Robots;

	fawkes::Mutex PlanMutex;
	fawkes::Time LastPlan;
	std::unordered_map<std::string, RobotPlan> Plan;
	std::size_t PlanElements;

	bool Unsat;

	bool UpdateNavgraphDistances;
	std::vector<Clingo::Symbol> NavgraphDistances;
	//As long as we don't have nodes for the zones we need a multimap.
	std::unordered_multimap<std::string, Clingo::Symbol> NavgraphNodesForASP;
	static constexpr auto NodePropertyASP = "ASP-Location";

	fawkes::Mutex RequestMutex;
	InterruptSolving Interrupt;
	bool SentCancel;
	std::vector<GroundRequest> Requests;

	fawkes::Mutex RobotsMutex;
	std::unordered_map<std::string, RobotInformation> RobotInformations;
	std::unordered_map<std::pair<Clingo::Symbol, unsigned int>, GroundRequest> RobotTaskBegin;
	std::unordered_map<std::pair<Clingo::Symbol, unsigned int>, GroundRequest> RobotTaskEnd;
	std::unordered_map<std::pair<Clingo::Symbol, unsigned int>, GroundRequest> RobotTaskUpdate;

	fawkes::Mutex SymbolMutex;
	Clingo::SymbolVector Symbols;
	bool NewSymbols;

	void fillNavgraphNodesForASP(void);
	void graph_changed(void) noexcept override final;

	void constructClingo(void);
	void initClingo(void);
	void loopClingo(void);
	void finalizeClingo(void);
	bool interruptSolving(void) const noexcept;
	void loadFilesAndGroundBase(fawkes::MutexLocker& locker);

	void updateNavgraphDistances(void);
	void setPast(void);

	void queueGround(GroundRequest&& request, const InterruptSolving interrupt = InterruptSolving::Not);
	void releaseExternals(RobotInformation &info, const bool lock = true);

	unsigned int realGameTimeToAspGameTime(const unsigned int realGameTime) const noexcept;
	unsigned int aspGameTimeToRealGameTime(const unsigned int aspGameTime) const noexcept;

	bool newModel(void);
	void solvingFinished(const Clingo::SolveResult& result);
	void groundFunctions(const Clingo::Location& loc, char const *name, const Clingo::SymbolSpan& arguments,
		Clingo::SymbolSpanCallback& retFunction);

	void setTeam(void);
	void unsetTeam(void);

	void newTeamMate(const std::string& mate, const RobotInformation& info);
	void deadTeamMate(const std::string& mate);

	void addZoneToExplore(const long zone);

	void robotBegunWithTask(const std::string& robot, const std::string& task, const unsigned int time);

	void beaconCallback(const mongo::BSONObj document);
	void gameTimeCallback(const mongo::BSONObj document);
	void teamColorCallback(const mongo::BSONObj document);
	void zonesCallback(const mongo::BSONObj document);

	void initPlan(void);
	void loopPlan(void);
	void updatePlanDB(const std::string& robot, const int elementIndex, const PlanElement& element);
	void removeFromPlanDB(const std::string& robot, const PlanElement& element);

	void planFeedbackCallback(const mongo::BSONObj document);

	public:
	AspPlanerThread(void);
	~AspPlanerThread(void);

	void init(void) override;
	void loop(void) override;
	void finalize(void) override;
//	bool prepare_finalize_user() override;

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	protected: void run() override { Thread::run(); return; }
};

#endif
