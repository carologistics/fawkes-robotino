
/***************************************************************************
 *  aps_agent_thread.cpp -  ASP-based agent main thread
 *
 *  Created on Mon Aug 15 17:39:02 2016
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

#include "asp_agent_thread.h"

using namespace fawkes;

/** @class AspAgentThread "asp_agent_thread.h"
 * The thread to start and control the Asp agent.
 */

/** Constructor. */
AspAgentThread::AspAgentThread() : Thread("AspAgentThread", Thread::OPMODE_WAITFORWAKEUP),
		BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK), aspCommon::RefboxComm(logger, config)
{
	LoggingComponent = name();
	return;
}

/** Destructor. */
AspAgentThread::~AspAgentThread()
{
	return;
}

void
AspAgentThread::init()
{
	logger->log_info(LoggingComponent, "Initialize ASP Agent");
	initRefboxComm();
	activateBeacon();
	openPublic();
	return;
}

void
AspAgentThread::loop()
{
	sendPeriodicMessages();
	return;
}

void
AspAgentThread::finalize()
{
	closePublic();
	logger->log_info(LoggingComponent, "Finalize ASP Agent");
	return;
}

//bool
//AspAgentThread::prepare_finalize_user()
//{
//	logger->log_info(LoggingComponent, "Prepare Finalize ASP Agent");
//	return true;
//}

