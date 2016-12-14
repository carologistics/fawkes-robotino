
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

#include "asp_planer_types.h"

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
	unsigned int MaxOrders;
	unsigned int MaxQuantity;

	unsigned int LookAhaed;
	std::unordered_map<std::string, unsigned int> NextTick;
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

	std::vector<RingColorInformation> RingColors;
	std::vector<OrderInformation> Orders;

	fawkes::Mutex RobotsMutex;
	std::unordered_map<std::string, RobotInformation> RobotInformations;
	std::unordered_map<std::pair<Clingo::Symbol, unsigned int>, GroundRequest> RobotTaskBegin;
	std::unordered_map<std::pair<Clingo::Symbol, unsigned int>, GroundRequest> RobotTaskUpdate;
	std::unordered_map<unsigned int, GroundRequest> TaskSuccess;

	fawkes::Mutex SymbolMutex;
	Clingo::SymbolVector Symbols;
	bool NewSymbols;

	void fillNavgraphNodesForASP(void);
	void graph_changed(void) noexcept override final;

	void constructClingo(void);
	void initClingo(void);
	void resetClingo(fawkes::MutexLocker& aspLocker, fawkes::MutexLocker& reqLocker);
	void loopClingo(void);
	void finalizeClingo(void);
	bool interruptSolving(void) const noexcept;
	void loadFilesAndGroundBase(fawkes::MutexLocker& locker);

	void updateNavgraphDistances(void);
	void setPast(std::vector<GroundRequest>& requests);

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

	void setRingColor(const RingColorInformation& info);
	void addOrder(const OrderInformation& order);

	void foundAMachine(void);
	void robotBegunWithTask(const std::string& robot, const std::string& task, unsigned int time);
	void robotUpdatesTaskTimeEstimation(const std::string& robot, const std::string& task, unsigned int time,
		unsigned int end);
	void robotFinishedTask(const std::string& robot, const std::string& task, unsigned int time);
	void taskWasFailure(const std::string& task, unsigned int time);

	void beaconCallback(const mongo::BSONObj document);
	void gameTimeCallback(const mongo::BSONObj document);
	void orderCallback(const mongo::BSONObj document);
	void ringColorCallback(const mongo::BSONObj document);
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
