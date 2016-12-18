
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

	bool Unsat;

	unsigned int ExplorationTime;
	unsigned int DeliverProductTaskDuration;
	unsigned int FetchProductTaskDuration;
	unsigned int LookAhaed;
	unsigned int MaxDriveDuration;
	unsigned int MaxOrders;
	unsigned int MaxQuantity;
	unsigned int MaxTaskDuration;
	std::vector<std::string> PossibleRobots;
	unsigned int PrepareCSTaskDuration;
	unsigned int ProductionEnd;
	unsigned int TimeResolution;

	const std::vector<std::string> BaseColors = {"RED", "BLACK", "SILVER"};
	const std::string SpecialBaseColor = "TRANSPARENT";
	std::vector<CapColorInformation> CapColors;

	fawkes::Mutex WorldMutex;
	unsigned int GameTime;
	std::vector<OrderInformation> Orders;
	std::vector<RingColorInformation> RingColors;
	std::unordered_map<std::string, RobotInformation> Robots;
	std::vector<unsigned int> ZonesToExplore;

	static constexpr auto NodePropertyASP = "ASP-Location";
	fawkes::Mutex NavgraphDistanceMutex;
	//As long as we don't have nodes for the zones we need a multimap.
	std::unordered_multimap<std::string, Clingo::Symbol> NavgraphNodesForASP;
	bool UpdateNavgraphDistances;
	std::vector<Clingo::Symbol> NavgraphDistances;

	/*
	fawkes::Mutex TaskLocationMutex;
	Clingo::Symbol DeliveryLocation;
	std::vector<Clingo::Symbol> RingLocations;
	std::vector<Clingo::Symbol> CapLocations;
	std::vector<Clingo::Symbol> BaseLocations;
	std::vector<Clingo::Symbol> GetLocations;
	std::vector<Clingo::Symbol> Tasks;
	std::vector<Clingo::Symbol> RingTasks[3];
	std::vector<Clingo::Symbol> CapTasks;
	*/

	/*
	std::unordered_map<std::string, unsigned int> NextTick;
	unsigned int Horizon;
	unsigned int Past;
	fawkes::Time SolvingStarted;
	fawkes::Time LastModel;
	unsigned int MachinesFound;
	bool StillNeedExploring;
	bool CompleteRestart;

	fawkes::Mutex PlanMutex;
	fawkes::Time LastPlan;
	std::unordered_map<std::string, RobotPlan> Plan;
	std::size_t PlanElements;

	fawkes::Mutex RobotsMutex;
	std::unordered_map<std::string, RobotInformation> RobotInformations;
	std::unordered_map<std::pair<Clingo::Symbol, unsigned int>, GroundRequest> RobotTaskBegin;
	std::unordered_map<std::pair<Clingo::Symbol, unsigned int>, GroundRequest> RobotTaskUpdate;
	std::unordered_multimap<unsigned int, GroundRequest> TaskSuccess;

	fawkes::Mutex SymbolMutex;
	Clingo::SymbolVector Symbols;
	bool NewSymbols;
	*/

	fawkes::Mutex RequestMutex;
	InterruptSolving Interrupt;
	bool SentCancel;
	std::vector<Clingo::Part> GroundRequests;
	std::vector<Clingo::Symbol> ReleaseRequests;
	std::vector<Clingo::Symbol> AssignRequests;

	bool ProgramGrounded;
	mutable fawkes::Mutex SolvingMutex;
	fawkes::Time LastModel;
	fawkes::Time SolvingStarted;
	bool NewSymbols;
	Clingo::SymbolVector Symbols;

	mutable fawkes::Mutex PlanMutex;
	fawkes::Time LastPlan;
	unsigned int StartSolvingGameTime;

	void loadConfig(void);

	void graph_changed(void) noexcept override final;
	void fillNavgraphNodesForASP(const bool lockWorldMutex);
	void updateNavgraphDistances(void);
	Clingo::Symbol nearestLocation(const float x, const float y);

	void initClingo(void);
	void finalizeClingo(void);
	void loopClingo(void);

	void queueGround(Clingo::Part&& part, const InterruptSolving interrupt = InterruptSolving::Not);
	void queueRelease(Clingo::Symbol&& atom, const InterruptSolving interrupt = InterruptSolving::Not);
	void queueAssign(Clingo::Symbol&& atom, const InterruptSolving interrupt = InterruptSolving::Not);
	void setInterrupt(const InterruptSolving interrupt, const bool lock = true);
	bool shouldInterrupt(void) const;

	bool newModel(void);
	void solvingFinished(const Clingo::SolveResult& result);
	void groundFunctions(const Clingo::Location& loc, char const *name, const Clingo::SymbolSpan& arguments,
		Clingo::SymbolSpanCallback& retFunction);

	void setTeam(void);
	void unsetTeam(void);

	void addOrderToASP(const OrderInformation& order);
	void addRingColorToASP(const RingColorInformation& color);

	void addZoneToExplore(const unsigned int zone);
	void releaseZone(const unsigned int zone);

	void initPlan(void);
	void loopPlan(void);
	void updatePlanDB(const std::string& robot, const int elementIndex, const PlanElement& element);
	void removeFromPlanDB(const std::string& robot, const PlanElement& element);

	void planFeedbackCallback(const mongo::BSONObj document);
	/*
	void resetClingo(fawkes::MutexLocker& aspLocker, fawkes::MutexLocker& reqLocker);
	bool interruptSolving(void) const noexcept;

	void setPast(std::vector<GroundRequest>& requests);

	void queueGround(GroundRequest&& request, const InterruptSolving interrupt = InterruptSolving::Not);
	void releaseExternals(RobotInformation &info, const bool lock = true);

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
	*/

	unsigned int realGameTimeToAspGameTime(const unsigned int realGameTime) const noexcept;
	unsigned int aspGameTimeToRealGameTime(const unsigned int aspGameTime) const noexcept;

	void beaconCallback(const mongo::BSONObj document);
	void gameTimeCallback(const mongo::BSONObj document);
	void orderCallback(const mongo::BSONObj document);
	void ringColorCallback(const mongo::BSONObj document);
	void teamColorCallback(const mongo::BSONObj document);
	void zonesCallback(const mongo::BSONObj document);

	public:
	AspPlanerThread(void);
	~AspPlanerThread(void);

	void init(void) override;
	void loop(void) override;
	void finalize(void) override;

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	protected: void run() override { Thread::run(); return; }
};

#endif
