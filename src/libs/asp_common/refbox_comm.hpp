/***************************************************************************
 *  refbox_comm.hpp - Refbox communication for the asp planer and agent
 *
 *  Created: Sat Aug 20 23:02:15 2016
 *  Copyright  2016  Björn Schäpers
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __LIBS_ASP_COMMON_REFBOX_COMM_HPP_
#define __LIBS_ASP_COMMON_REFBOX_COMM_HPP_

#include <boost/asio/ip/udp.hpp>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <core/threading/mutex.h>

namespace google {
	namespace protobuf {
class Message;
	}
}

namespace llsf_msgs {
class BeaconSignal;
}

namespace protobuf_comm {
class MessageRegister;
class ProtobufBroadcastPeer;
}

namespace fawkes {

class Configuration;
class Logger;
class Mutex;

	namespace aspCommon {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;

namespace StandardTimings {
	static constexpr auto Ack         = std::chrono::milliseconds(10);
	static constexpr auto Burst       = std::chrono::milliseconds(50);
	static constexpr auto High        = std::chrono::milliseconds(100);
	static constexpr auto Medium      = std::chrono::milliseconds(250);
	static constexpr auto Low         = std::chrono::milliseconds(500);
	static constexpr auto Beacon      = std::chrono::seconds(1);
	static constexpr auto Report      = std::chrono::seconds(2);
	static constexpr auto ReportBurst = std::chrono::milliseconds(100);
}

struct MessageStruct
{
	google::protobuf::Message *Message;
	std::chrono::milliseconds Delay;
	unsigned short Counter = 0;
	std::chrono::milliseconds DelayAfterCounter;
	//The last digit of the id is the robot number of the sender, this way the message ids are unique for all robots
	uint32_t MessageID = 0;
	uint32_t NeedsAck = 0;
	std::unordered_set<uint32_t> ReceivedAcks;
	std::function<void(void)> Function;
	TimePoint LastSend;
	bool DeleteMessage = false;

	MessageStruct(google::protobuf::Message *m, const std::chrono::milliseconds& delay,
		const bool deleteMessage = false) noexcept;
	MessageStruct(google::protobuf::Message *m, const std::chrono::milliseconds& delay,
		const std::function<void(void)>& f) noexcept;
	MessageStruct(google::protobuf::Message *m, const std::chrono::milliseconds& delay, const unsigned short count)
		noexcept;
	MessageStruct(const MessageStruct&) = delete;
	MessageStruct(MessageStruct&& that) = default;

	~MessageStruct(void);

	MessageStruct& operator=(const MessageStruct&) = delete;
	MessageStruct& operator=(MessageStruct&& that) = default;

	void changeDelay(const std::chrono::milliseconds& delay, const unsigned short count = 10) noexcept;
};

class RefboxComm
{
	private:
	mutable Mutex ChannelMutex;
	protobuf_comm::ProtobufBroadcastPeer *PublicChannel;
	protobuf_comm::ProtobufBroadcastPeer *TeamChannel;

	std::string TeamName;
	uint32_t Number;

	mutable Mutex MessageMutex;
	std::vector<MessageStruct> PeriodicMessages;
	uint32_t NextMessageID;
	enum { ACK_PUFFER_SIZE = 25 };
	uint32_t AckedMessages[ACK_PUFFER_SIZE];
	uint32_t *NextAckPos;

	std::unordered_map<uint32_t, TimePoint> TeamMates;
	TimePoint Now;

	void sendError(const std::string& msg);
	void recvError(const boost::asio::ip::udp::endpoint& endpoint, const std::string& msg);

	void recvPublicCommon(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id,
		const uint16_t msg_type, const std::shared_ptr<google::protobuf::Message>& msg);
	void recvTeamCommon(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id,
		const uint16_t msg_type, const std::shared_ptr<google::protobuf::Message>& msg);

	void registerTeamMate(const uint32_t number);
	bool checkAcks(const MessageStruct& message);
	bool checkTeamMates(void);

	void setupPeer(protobuf_comm::ProtobufBroadcastPeer *peer);
	void openTeam(const unsigned short sendPort, const unsigned short recvPort);

	protected:
	protobuf_comm::MessageRegister* const PublicRegister;
	protobuf_comm::MessageRegister* const TeamRegister;

	llsf_msgs::BeaconSignal* const Beacon;
	MessageStruct *BeaconStruct;

	Logger*& Log;
	Configuration*& Config;
	const char *ConfigPrefix;

	virtual void recvPublic(const boost::asio::ip::udp::endpoint &endpoint, const uint16_t comp_id,
		const uint16_t msg_type, const std::shared_ptr<google::protobuf::Message> &msg) = 0;
	virtual void recvTeam(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id,
		const uint16_t msg_type, const std::shared_ptr<google::protobuf::Message>& msg) = 0;

	bool ackMessageID(const uint32_t id);
	uint32_t setMessageID(MessageStruct& message, const uint32_t needAck = 10);

	virtual void updateBeacon(void);

	virtual void deadTeamMate(const uint32_t mate);
	virtual void newTeamMate(const uint32_t mate);

	public:
	const char *LoggingComponent;

	RefboxComm(Logger*& log, Configuration*& config);
	virtual ~RefboxComm(void);

	void initRefboxComm(void);

	bool checkPublic(void);
	bool publicOpen(void) const noexcept;
	void openPublic(void);
	void closePublic(void) noexcept;

	bool teamOpen(void) const noexcept;
	void closeTeam(void) noexcept;

	void setConfigPrefix(const char *prefix);

	void activateBeacon(void);

	void sendPeriodicMessages(void);

	uint32_t number(void) const noexcept;
};

	} //namespace aspCommon
} //namespace fawkes

#endif
