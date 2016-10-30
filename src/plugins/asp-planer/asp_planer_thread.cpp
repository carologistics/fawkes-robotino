
/***************************************************************************
 *  aps_planer_thread.cpp -  ASP-based planer main thread
 *
 *  Created on Thu Aug 18 04:20:02 2016
 *  Copyright (C) 2016 by Björn Schäpers
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

using namespace fawkes;

/** @class AspAgentThread "asp_agent_thread.h"
 * The thread to start and control the Asp agent.
 */

/** Constructor. */
AspPlanerThread::AspPlanerThread(void) : Thread("AspPlanerThread", Thread::OPMODE_WAITFORWAKEUP),
		BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK), ASPAspect("ASPAgent", "ASP Agent"),
		aspCommon::RefboxComm(logger, config),
		ClingoDebug(false), MoreModels(false), ClingoMutex(Mutex::RECURSIVE), Solving(false),
		LastTick(0), LastGameTime(0), Horizon(0)
{
	constructRefboxComm();
	constructClingo();
	return;
}

/** Destructor. */
AspPlanerThread::~AspPlanerThread(void)
{
	return;
}

void
AspPlanerThread::init()
{
	logger->log_info(LoggingComponent, "Initialize ASP Planer");
	initRefboxComm();
	initClingo();
	return;
}

void
AspPlanerThread::loop()
{
	loopClingo();
	sendPeriodicMessages();
	return;
}

void
AspPlanerThread::finalize()
{
	logger->log_info(LoggingComponent, "Finalize ASP Planer");
	finalizeRefboxComm();
	finalizeClingo();
	return;
}

//bool
//AspAgentThread::prepare_finalize_user()
//{
//	logger->log_info(LoggingComponent, "Prepare Finalize ASP Planer");
//	return true;
//}

