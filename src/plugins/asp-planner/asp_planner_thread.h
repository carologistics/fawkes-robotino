
/***************************************************************************
 *  asp_planner_thread.h - ASP-based planner plugin
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

#include "asp_planner_types.h"

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>
#include <plugins/asp/aspect/asp.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>

#include <atomic>
#include <bsoncxx/document/view.hpp>
#include <unordered_set>

class AspPlannerThread : public fawkes::Thread,
                         public fawkes::ConfigurableAspect,
                         public fawkes::LoggingAspect,
                         public fawkes::ASPAspect,
                         public fawkes::RobotMemoryAspect,
                         public fawkes::NavGraphAspect,
                         public fawkes::NavGraph::ChangeListener
{
private:
	const char *const           LoggingComponent;
	const char *const           ConfigPrefix;
	std::vector<EventTrigger *> RobotMemoryCallbacks;
	const char *                TeamColor;

	int Unsat;

	int                                  ExplorationTime;
	int                                  DeliverProductTaskDuration;
	int                                  FetchProductTaskDuration;
	int                                  LookAhaed;
	int                                  MaxDriveDuration;
	int                                  MaxOrders;
	int                                  MaxProducts;
	int                                  MaxQuantity;
	int                                  MaxTaskDuration;
	int                                  MaxWorkingDuration;
	std::vector<std::string>             PossibleRobots;
	int                                  PrepareCSTaskDuration;
	int                                  ProductionEnd;
	int                                  TimeResolution;
	std::unordered_map<std::string, int> WorkingDurations;

	const std::vector<std::string>   BaseColors       = {"RED", "BLACK", "SILVER"};
	const std::string                SpecialBaseColor = "TRANSPARENT";
	std::vector<CapColorInformation> CapColors;

	fawkes::Mutex                                       WorldMutex;
	int                                                 GameTime;
	std::unordered_map<int, OrderInformation>           Orders;
	std::vector<RingColorInformation>                   RingColors;
	std::unordered_map<std::string, RobotInformation>   Robots;
	std::unordered_map<std::string, MachineInformation> Machines;
	std::vector<Product>                                Products;
	bool                                                ReceivedZonesToExplore;
	std::vector<int>                                    ZonesToExplore;
	std::unordered_map<std::pair<int, int>, OrderTasks> OrderTaskMap;

	static constexpr auto NodePropertyASP = "ASP-Location";
	fawkes::Mutex         NavgraphDistanceMutex;
	// As long as we don't have nodes for the zones we need a multimap.
	std::unordered_multimap<std::string, Clingo::Symbol> NavgraphNodesForASP;
	std::unordered_set<std::string>                      NodesToFind;
	bool                                                 UpdateNavgraphDistances;
	std::vector<Clingo::Symbol>                          NavgraphDistances;

	Clingo::Symbol DeliveryLocation;
	Clingo::Symbol RingLocations[2];
	Clingo::Symbol CapLocations[2];

	using GroundRequest = std::pair<const char *, Clingo::SymbolVector>;

	std::atomic<InterruptSolving> Interrupt;
	std::atomic<const char *>     InterruptReason;

	fawkes::Mutex               RequestMutex;
	bool                        SentCancel;
	std::vector<GroundRequest>  GroundRequests;
	std::vector<Clingo::Symbol> ReleaseRequests;
	std::vector<Clingo::Symbol> AssignRequests;

	bool ProgramGrounded;
	bool ProductionStarted;

	mutable fawkes::Mutex SolvingMutex;
	TimePoint             LastModel;
	TimePoint             SolvingStarted;
	bool                  NewSymbols;
	Clingo::SymbolVector  Symbols;
	int                   StartSolvingGameTime;

	mutable fawkes::Mutex                           PlanMutex;
	TimePoint                                       LastPlan;
	int                                             PlanGameTime;
	std::unordered_map<std::string, RobotPlan>      Plan;
	std::unordered_map<Clingo::Symbol, std::string> LocationInUse;

	void loadConfig(void);

	void           graph_changed(void) noexcept override final;
	void           fillNavgraphNodesForASP(const bool lockWorldMutex);
	void           updateNavgraphDistances(void);
	Clingo::Symbol nearestLocation(const float x, const float y);

	void initClingo(void);
	void finalizeClingo(void);
	void loopClingo(void);

	void queueGround(GroundRequest &&       request,
	                 const char *           reason,
	                 const InterruptSolving interrupt = InterruptSolving::Not);
	void queueRelease(Clingo::Symbol &&      atom,
	                  const char *           reason,
	                  const InterruptSolving interrupt = InterruptSolving::Not);
	void queueAssign(Clingo::Symbol &&      atom,
	                 const char *           reason,
	                 const InterruptSolving interrupt = InterruptSolving::Not);
	void setInterrupt(const InterruptSolving interrupt, const char *reason);
	bool shouldInterrupt(void);

	bool newModel(void);
	void solvingFinished(const Clingo::SolveResult &result);
	void groundFunctions(const Clingo::Location &    loc,
	                     char const *                name,
	                     const Clingo::SymbolSpan &  arguments,
	                     Clingo::SymbolSpanCallback &retFunction);

	void setTeam(void);
	void unsetTeam(void);

	void addOrderToASP(const OrderInformation &order);
	void addRingColorToASP(const RingColorInformation &color);

	void addZoneToExplore(const int zone);
	void releaseZone(const int zone, const bool removeAndFillNodes);

	void checkForInterruptBasedOnTimeOffset(int offset);
	void robotBegunWithTask(const std::string &robot,
	                        const std::string &task,
	                        const int          begin,
	                        const int          end);
	void
	robotUpdatesTaskTimeEstimation(const std::string &robot, const std::string &task, const int end);
	void robotFinishedTask(const std::string &robot,
	                       const std::string &task,
	                       const int          end,
	                       const bool         success);

	void initPlan(void);
	void loopPlan(void);
	void
	insertPlanElement(const std::string &robot, const int elementIndex, const PlanElement &element);
	void updatePlan(const std::string &robot, const int elementIndex, const PlanElement &element);
	void
	updatePlanTiming(const std::string &robot, const int elementIndex, const PlanElement &element);
	void removeFromPlanDB(const std::string &robot, const int elementIndex);
	void tellRobotToStop(const std::string &robot);

	void planFeedbackCallback(const bsoncxx::document::view &document);

	int realGameTimeToAspGameTime(const int realGameTime) const noexcept;
	int aspGameTimeToRealGameTime(const int aspGameTime) const noexcept;

	void beaconCallback(const bsoncxx::document::view &document);
	void gameTimeCallback(const bsoncxx::document::view &document);
	void machineCallback(const bsoncxx::document::view &document);
	void orderCallback(const bsoncxx::document::view &document);
	void ringColorCallback(const bsoncxx::document::view &document);
	void teamColorCallback(const bsoncxx::document::view &document);
	void zonesCallback(const bsoncxx::document::view &document);

public:
	AspPlannerThread(void);
	~AspPlannerThread(void);

	void init(void) override;
	void loop(void) override;
	void finalize(void) override;

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	void
	run() override
	{
		Thread::run();
		return;
	}
};

#endif
