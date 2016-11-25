
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
#include <plugins/asp/aspect/asp.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>

#include <clingo.hh>

#include <unordered_map>
#include <vector>

class EventTrigger;

enum class InterruptSolving : unsigned short {
	Not = 0,
	Normal,
	Critical
};

struct GroundRequest
{
	const char *Name;
	Clingo::SymbolVector Params;
	bool AddTick;
};

struct RobotInformation
{
	fawkes::Time LastSeen;
	double X;
	double Y;
};

class AspPlanerThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ASPAspect,
  public fawkes::RobotMemoryAspect
{
	private:
	const char* const LoggingComponent;
	const char* const ConfigPrefix;
	std::vector<EventTrigger*> RobotMemoryCallbacks;
	bool TeamColorSet;

	bool MoreModels;
	unsigned int ExplorationTime;

	unsigned int LookAhaed;
	unsigned int LastTick;
	unsigned int GameTime;
	unsigned int Horizon;
	fawkes::Time LastModel;

	fawkes::Mutex RequestMutex;
	InterruptSolving Interrupt;
	std::vector<GroundRequest> Requests;

	fawkes::Mutex RobotsMutex;
	std::unordered_map<std::string, RobotInformation> Robots;

	fawkes::Mutex SymbolMutex;
	Clingo::SymbolVector Symbols;
	bool NewSymbols;

	void constructClingo(void);
	void initClingo(void);
	void loopClingo(void);
	void finalizeClingo(void);

	void queueGround(GroundRequest&& request, const InterruptSolving interrupt = InterruptSolving::Not);

	bool newModel(void);
	void solvingFinished(const Clingo::SolveResult& result);

	void setTeam(const bool cyan);
	void unsetTeam(void);

	void newTeamMate(const std::string& mate);
	void deadTeamMate(const std::string& mate);

	void beaconCallback(const mongo::BSONObj document);
	void gameTimeCallback(const mongo::BSONObj document);
	void teamColorCallback(const mongo::BSONObj document);

	public:
	AspPlanerThread(void);
	~AspPlanerThread(void);

	void init() override;
	void loop() override;
	void finalize() override;
//	bool prepare_finalize_user() override;

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	protected: void run() override { Thread::run(); return; }
};

#endif
