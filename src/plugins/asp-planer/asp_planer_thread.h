
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
#include <asp_common/refbox_comm.hpp>
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
  public fawkes::ASPAspect,
  public fawkes::aspCommon::RefboxComm
{
	private:
	bool ClingoDebug;
	bool MoreModels;
	fawkes::Mutex ClingoMutex;
	bool Solving;
	unsigned int LastTick;
	unsigned int LastGameTime;
	unsigned int Horizon;

	fawkes::Mutex RequestMutex;
	std::vector<GroundRequest> Requests;

	void constructRefboxComm(void);
	void initRefboxComm(void);
	void finalizeRefboxComm(void);

	void constructClingo(void);
	void initClingo(void);
	void loopClingo(void);
	void finalizeClingo(void);

	void resetClingo(void);

	void queueGround(GroundRequest&& request);
	void ground(const Clingo::PartSpan& parts);
	void solve(void);

	bool newModel(const Clingo::Model& model);
	void solvingFinished(const Clingo::SolveResult& result);

	protected:
	void recvPublic(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id, const uint16_t msg_type,
		const std::shared_ptr<google::protobuf::Message>& msg) override;
	void recvTeam(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id, const uint16_t msg_type,
		const std::shared_ptr<google::protobuf::Message>& msg) override;

	void setTeam(const bool cyan) override;
	void unsetTeam(void) override;

	void newTeamMate(const uint32_t mate) override;
	void deadTeamMate(const uint32_t mate) override;

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
