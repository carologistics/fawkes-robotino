
/***************************************************************************
 *  asp_agent_thread.h - ASP-based agent plugin
 *
 *  Created on Mon Aug 15 17:39:02 2016
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

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <asp_common/refbox_comm.hpp>
#include <core/threading/thread.h>

class AspAgentThread
: public fawkes::Thread,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::TransformAspect,
  public fawkes::aspCommon::RefboxComm
{
	private:
	uint32_t Planer;

	void constructRefboxComm(void);
	void initRefboxComm(void);
	void finalizeRefboxComm(void);

	protected:
	void recvPublic(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id, const uint16_t msg_type,
		const std::shared_ptr<google::protobuf::Message>& msg) override;
	void recvTeam(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id, const uint16_t msg_type,
		const std::shared_ptr<google::protobuf::Message>& msg) override;

	void updateBeacon(void) override;

	void deadTeamMate(const uint32_t mate) override;

	public:
	AspAgentThread();
	~AspAgentThread();

	void init() override;
	void loop() override;
	void finalize() override;
//	bool prepare_finalize_user() override;

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	protected: void run() override { Thread::run(); return; }
};

#endif
