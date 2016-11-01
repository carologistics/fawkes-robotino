
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
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <plugins/asp/aspect/asp.h>

#include <clingo.hh>

struct GroundRequest
{
	const char *Name;
	Clingo::SymbolVector Params;
	bool AddTick;
};

class AspPlanerThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::ASPAspect
{
	private:
	const char* const LoggingComponent;
	const char* const ConfigPrefix;

	bool MoreModels;
	unsigned int ExplorationTime;

	unsigned int LastTick;
	unsigned int GameTime;
	unsigned int LastGameTime;
	unsigned int Horizon;

	fawkes::Mutex RequestMutex;
	std::vector<GroundRequest> Requests;

	fawkes::Mutex SymbolMutex;
	Clingo::SymbolVector Symbols;
	bool NewSymbols;

	void constructClingo(void);
	void initClingo(void);
	void loopClingo(void);
	void finalizeClingo(void);

	void queueGround(GroundRequest&& request);

	bool newModel(void);
	void solvingFinished(const Clingo::SolveResult& result);

	void setTeam(const bool cyan);
	void unsetTeam(void);

	void newTeamMate(const uint32_t mate);
	void deadTeamMate(const uint32_t mate);

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
