/***************************************************************************
 *  asp_agent_refbox_comm.cpp - ASP-based agent plugin refbox communication
 *
 *  Created on Mon Aug 22 15:25:02 2016
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

#include "asp_agent_thread.h"

#include <asp_msgs/Beacon.pb.h>
#include <llsf_msgs/BeaconSignal.pb.h>
#include <llsf_msgs/GameState.pb.h>

using namespace fawkes;

/**
 * @var AspAgentThread::Planer
 * @brief The id of the current planer.
 */

/**
 * @brief Takes care of everything regarding refbox communication in the constructor.
 */
void AspAgentThread::constructRefboxComm(void)
{
	LoggingComponent = name();
	return;
}

/**
 * @brief Takes care of everything regarding refbox communication in init().
 */
void AspAgentThread::initRefboxComm(void)
{
	aspCommon::RefboxComm::initRefboxComm();
	activateBeacon();
	openPublic();
	return;
}

/**
 * @brief Takes care of everything regarding refbox communication in finalize().
 */
void AspAgentThread::finalizeRefboxComm(void)
{
	closePublic();
	return;
}

/**
 * @brief Handles public messages.
 */
void AspAgentThread::recvPublic(const boost::asio::ip::udp::endpoint& /*endpoint*/, const uint16_t comp_id,
		const uint16_t msg_type, const std::shared_ptr<google::protobuf::Message>& msg)
{
	assert(comp_id == 2000);
	switch ( msg_type )
	{
		case llsf_msgs::GameState_CompType_MSG_TYPE :
		{
			auto game = std::static_pointer_cast<llsf_msgs::GameState>(msg);
			break;
		} //case llsf_msgs::GameState_CompType_MSG_TYPE
		default : break;
	} //switch ( msg_type )
	return;
}

/**
 * @brief Handles team messages.
 */
void AspAgentThread::recvTeam(const boost::asio::ip::udp::endpoint& /*endpoint*/, const uint16_t comp_id,
		const uint16_t msg_type, const std::shared_ptr<google::protobuf::Message>& msg)
{
	switch ( comp_id )
	{
		case 2000 :
		{
			switch ( msg_type )
			{
				default : break;
			} //switch ( msg_type )
			break;
		} //case 2000
		case 4711 :
		{
			switch ( msg_type )
			{
				case asp_msgs::PlanerBeacon_CompType_MSG_TYPE :
				{
					auto beacon = std::static_pointer_cast<asp_msgs::PlanerBeacon>(msg);
					const auto planer = beacon->number();
					if ( planer != Planer )
					{
						if ( Planer )
						{
							Log->log_error(LoggingComponent,
								"We think %d is the planer, but now %d claims to be one. We now accept him.",
								Planer, planer);
						} //if ( Planer )
						else
						{
							Log->log_warn(LoggingComponent, "Detected planer: %d", planer);
						} //else -> if ( Planer )
						Planer = planer;
					} //if ( planer != Planer )
					break;
				} //case asp_msgs::PlanerBeacon_CompType_MSG_TYPE
				default : break;
			} //switch ( msg_type )
			break;
		} //case 4711
		default : break;
	} //switch ( comp_id )
	return;
}

/**
 * @brief Updates the robot position in the beacon.
 */
void AspAgentThread::updateBeacon(void)
{

	return;
}

/**
 * @brief Handles a dead planer.
 */
void AspAgentThread::deadTeamMate(const uint32_t mate)
{
	if ( mate == Planer )
	{
		Log->log_warn(LoggingComponent, "We lost our planer.");
		Planer = 0;
	} //if ( mate == Planer )
	return;
}
