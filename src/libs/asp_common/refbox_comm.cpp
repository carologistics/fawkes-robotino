#include <cstring>

#include <asp_msgs/Ack.pb.h>
#include <asp_msgs/Beacon.pb.h>
#include <config/config.h>
#include <llsf_msgs/BeaconSignal.pb.h>
#include <llsf_msgs/ExplorationInfo.pb.h>
#include <llsf_msgs/GameState.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/MachineReport.pb.h>
#include <llsf_msgs/OrderInfo.pb.h>
#include <llsf_msgs/RingInfo.pb.h>
#include <llsf_msgs/RobotInfo.pb.h>
#include <llsf_msgs/VersionInfo.pb.h>
#include <logging/logger.h>
#include <protobuf_comm/message_register.h>
#include <protobuf_comm/peer.h>

#include "refbox_comm.hpp"

using namespace protobuf_comm;

namespace fawkes {
namespace aspCommon {

/**
 * @class MessageStruct
 * @brief Container for messages to send.
 *
 * @var MessageStruct::Message
 * @brief Pointer to the message.
 *
 * @var MessageStruct::Delay
 * @brief The delay between two sends.
 *
 * @var MessageStruct::Counter
 * @brief How often this messages will be send. (For exceptions see below.)
 *
 * @var MessageStruct::DelayAfterCounter
 * @brief If not equal to zero Delay will be set to this value after the Counter dropped to zero and we still wait for
 *        an ACK.
 *
 * @var MessageStruct::MessageID
 * @brief The ID of the message, only messages which needs ACKs need an ID.
 *
 * @var MessageStruct::NeedsAck
 * @brief If the message needs an ACK.
 *
 * 0 means no ACK is needed.
 * 10 means an ACK for each team member is needed.
 * 1 to 9 means only an ACK from team member X is needed.
 * Team member in this context are all robots but also additional comminucation partners from outside the field.
 *
 * @var MessageStruct::ReceivedAcks
 * @brief Stores all the ACKs we received for this message.
 *
 * @var MessageStruct::Function
 * @brief This function will be executed each time before the message is send. E.g. to update a timestamp in the
 *        message.
 *
 * @var MessageStruct::LastSend
 * @brief The time point at which the message was last send.
 *
 * @var MessageStruct::DeleteMessage
 * @brief If the message stored in Message should be deleted when deleting this struct.
 */

/**
 * @brief Constructor.
 */
MessageStruct::MessageStruct(google::protobuf::Message *m, const std::chrono::milliseconds& delay,
		const bool deleteMessage) noexcept : Message(m), Delay(delay), DeleteMessage(deleteMessage)
{
	return;
}

/**
 * @brief Constructor.
 */
MessageStruct::MessageStruct(google::protobuf::Message *m, const std::chrono::milliseconds& delay,
		const std::function<void(void)>& f) noexcept : Message(m), Delay(delay), Function(f)
{
	return;
}

/**
 * @brief Constructor.
 */
MessageStruct::MessageStruct(google::protobuf::Message *m, const std::chrono::milliseconds& delay,
		const unsigned short count) noexcept : Message(m), Delay(delay), Counter(count), DeleteMessage(true)
{
	return;
}

/**
 * @brief Destructor.
 */
MessageStruct::~MessageStruct(void)
{
	if ( DeleteMessage )
	{
		delete Message;
	}
	return;
}

/**
 * @brief Changes the delay, used for bursts in which the count will be set. After the counter drops to zero the old
 *        delay will be restored.
 * @param[in] delay The new delay.
 * @param[in] count For how many sendings the new delay should be used.
 */
void
MessageStruct::changeDelay(const std::chrono::milliseconds& delay, const unsigned short count) noexcept
{
	DelayAfterCounter = Delay;
	Delay = delay;
	Counter = count;
	return;
}

/**
 * @class RefboxComm
 * @brief Implements the common refbox communication of the agent and the planet.
 *
 * @var RefboxComm::ChannelMutex
 * @brief Protectes the channel members.
 *
 * @var RefboxComm::PublicChannel
 * @brief The public channel.
 *
 * @var RefboxComm::PrivateChannel
 * @brief The private channel.
 *
 * @var RefboxComm::TeamName
 * @brief The team name.
 *
 * @var RefboxComm::Number
 * @brief The number of this team member.
 *
 * 1 to 6 for the robot number or 8 respectively 9 for the planer.
 *
 * @var RefboxComm::ExplorationTime
 * @brief How long the exploration is.
 *
 * @var RefboxComm::GameStateMutex
 * @brief Protects GameState.
 *
 * @var RefboxComm::GameState
 * @brief A copy of the last game state we received.
 *
 * @var RefboxComm::MessageMutex
 * @brief Protects the access of PeriodicMessages and friends.
 *
 * @var RefboxComm::PeriodicMessages
 * @brief Contains all messages which will be send on a regular basis.
 *
 * @var RefboxComm::NextMessageID
 * @brief The id of the next message to create.
 *
 * @var RefboxComm::AckedMessages
 * @brief Contains all recently ack'ed message ids.
 *
 * @var RefboxComm::NextAckPos
 * @brief Pointer in AckedMessages, where to save the next id.
 *
 * @var RefboxComm::TeamMates
 * @brief Saves all known team mates and when we heard last of them.
 *
 * @var RefboxComm::Now
 * @brief A cached time point, will be updated everytime the periodic messages are sent.
 *
 * @var RefboxComm::PublicRegister
 * @brief The register for public messages.
 *
 * @var RefboxComm::PrivateRegister
 * @brief The register for private messages, register here all custom messages.
 *
 * @var RefboxComm::Beacon
 * @brief The beacon signal.
 *
 * @var RefboxComm::BeaconStruct
 * @brief The MessageStruct which contains the beacon as message.
 *
 * @var RefboxComm::Log
 * @brief The used logging facility.
 *
 * @var RefboxComm::Config
 * @brief The used configuration.
 *
 * @var RefboxComm::ConfigPrefix
 * @brief The prefix used for the configuration.
 *
 * @var RefboxComm::LoggingComponent
 * @brief Is used for logging, should be set by the user.
 *
 * @fn void RefboxComm::recvPublic(const boost::asio::ip::udp::endpoint&, const uint16_t,
		const uint16_t, const std::shared_ptr<google::protobuf::Message>&)
 * @brief Will be called when a message was send on the public channel.
 * @param[in] endpoint From which endpoint the message was send.
 * @param[in] comp_id The component id of the message.
 * @param[in] msg_type The type of the message.
 * @param[in] msg The message.
 *
 * The common and default behavior is handeld in RefboxComm::recvPublicCommon.
 *
 * @fn void RefboxComm::recvTeam(const boost::asio::ip::udp::endpoint&, const uint16_t,
		const uint16_t, const std::shared_ptr<google::protobuf::Message>&)
 * @brief Will be called when a message was send on the team channel.
 * @param[in] endpoint From which endpoint the message was send.
 * @param[in] comp_id The component id of the message.
 * @param[in] msg_type The type of the message.
 * @param[in] msg The message.
 *
 * The common and default behavior is handeld in RefboxComm::recvTeamCommon.
 *
 * @fn void RefboxComm::sendMessage(Args&&...)
 * @brief Queues a message in PeriodicMessages.
 * @param[in, out] message The message, will be moved.
 *
 */

/**
 * @brief Will be called when a sending error occured.
 * @param[in] msg The error message.
 */
void
RefboxComm::sendError(const std::string& msg)
{
	Log->log_warn(LoggingComponent, "Sending Error: %s", msg.c_str());
	return;
}

/**
 * @brief Will be called when a receiving error occured.
 * @param[in] endpoint From which endpoint the message was received.
 * @param[in] msg The error message.
 */
void
RefboxComm::recvError(const boost::asio::ip::udp::endpoint& endpoint, const std::string& msg)
{
	Log->log_warn(LoggingComponent, "Receiving Error from %s:%hu: %s", endpoint.address().to_string().c_str(),
		endpoint.port(), msg.c_str());
	return;
}

/**
 * @brief Handles the common receivement of public messages. I.e. opens the team channel.
 * @param[in] endpoint From which endpoint the message was send.
 * @param[in] comp_id The component id of the message.
 * @param[in] msg_type The type of the message.
 * @param[in] msg The message.
 */
void
RefboxComm::recvPublicCommon(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id,
		const uint16_t msg_type, const std::shared_ptr<google::protobuf::Message>& msg) {
	assert(comp_id == 2000);
	switch ( msg_type )
	{
		case llsf_msgs::GameState_CompType_MSG_TYPE :
		{
			auto game = std::static_pointer_cast<llsf_msgs::GameState>(msg);
			if ( !teamOpen() )
			{
				const auto prefixLen = std::strlen(ConfigPrefix);
				char buffer[prefixLen + 20];
				const auto suffix = buffer + prefixLen;
				std::strcpy(buffer, ConfigPrefix);
				if ( game->team_cyan() == TeamName )
				{
					Log->log_info(LoggingComponent, "Opening cyan team channel.");
					Beacon->set_team_color(llsf_msgs::Team::CYAN);

					unsigned short recvPort, sendPort;
					std::strcpy(suffix, "cyan-send-port");
					sendPort = Config->get_int(buffer);
					std::strcpy(suffix, "cyan-recv-port");
					recvPort = Config->get_int(buffer);

					openTeam(sendPort, recvPort);
					setTeam(true);
				} //if ( game->team_cyan() == TeamName )
				if ( game->team_magenta() == TeamName )
				{
					Log->log_info(LoggingComponent, "Opening magenta team channel.");
					Beacon->set_team_color(llsf_msgs::Team::MAGENTA);

					unsigned short recvPort, sendPort;
					std::strcpy(suffix, "magenta-send-port");
					sendPort = Config->get_int(buffer);
					std::strcpy(suffix, "magenta-recv-port");
					recvPort = Config->get_int(buffer);

					openTeam(sendPort, recvPort);
					setTeam(false);
				} //if ( game->team_magenta() == TeamName )
			} //if ( !teamOpen() )
			else
			{
				if ( game->team_cyan() != TeamName && game->team_magenta() != TeamName )
				{
					closeTeam();
					unsetTeam();
				} //if ( game->team_cyan() != TeamName && game->team_magenta() != TeamName )
			} //else -> if ( !teamOpen() )
			MutexLocker locker(&GameStateMutex);
			if ( GameState->state() != game->state() )
			{
				gameStateChanged(game->state(), GameState->state());
			} //if ( GameState->state() != game->state() )
			if ( GameState->phase() != game->phase() )
			{
				gamePhaseChanged(game->phase(), GameState->phase());
			} //if ( GameState->phase() != game->phase() )
			*GameState = *game;
			break;
		} //case llsf_msgs::GameState_CompType_MSG_TYPE
		case llsf_msgs::VersionInfo_CompType_MSG_TYPE :
		{
			auto version = std::static_pointer_cast<llsf_msgs::VersionInfo>(msg);
			constexpr auto targetVersion = "1.0.0";
			if ( version->version_string() != targetVersion )
			{
				Log->log_warn(LoggingComponent,
					"Different version number! Refbox Version: %s Roby Plugin is built for version: %s",
					version->version_string().c_str(), targetVersion);
			} //if ( version->version_string() != targetVersion )
			break;
		} //case llsf_msgs::VersionInfo_CompType_MSG_TYPE
		default : break;
	} //switch ( msg_type )
	recvPublic(endpoint, comp_id, msg_type, msg);
	return;
}

/**
 * @brief Handles the common receivement of team messages. Especially the ACK handling.
 * @param[in] endpoint From which endpoint the message was send.
 * @param[in] comp_id The component id of the message.
 * @param[in] msg_type The type of the message.
 * @param[in] msg The message.
 */
void
RefboxComm::recvTeamCommon(const boost::asio::ip::udp::endpoint& endpoint, const uint16_t comp_id,
		const uint16_t msg_type, const std::shared_ptr<google::protobuf::Message>& msg) {
	switch ( comp_id )
	{
		case 2000 :
		{
			switch ( msg_type )
			{
				case llsf_msgs::BeaconSignal_CompType_MSG_TYPE :
				{
					auto beacon = std::static_pointer_cast<llsf_msgs::BeaconSignal>(msg);
					registerTeamMate(beacon->number());
					break;
				} //case llsf_msgs::BeaconSignal_CompType_MSG_TYPE
				default : break;
			} //switch ( msg_type )
			break;
		} //case 2000
		case 4711 :
		{
			switch ( msg_type )
			{
				case asp_msgs::Ack_CompType_MSG_TYPE :
				{
					auto ack = std::static_pointer_cast<asp_msgs::Ack>(msg);
					const uint32_t id = ack->id();
					const uint32_t receiver = id % 10;

					registerTeamMate(ack->number());

					if ( receiver == number() )
					{
						MutexLocker locker(&MessageMutex);
						//This ack is for us!
						auto iter = std::find_if(PeriodicMessages.begin(), PeriodicMessages.end(),
							[id](const MessageStruct& m) noexcept
							{
								return m.MessageID == id;
							});
						if ( iter != PeriodicMessages.end() )
						{
							//And we are still waiting for an ack
							iter->ReceivedAcks.insert(ack->number());
							if ( checkAcks(*iter) )
							{
								PeriodicMessages.erase(iter);
							} //if ( checkAcks(*iter) )
						} //if ( iter != PeriodicMessages.end() )
					} //if ( receiver == number() )
					break;
				} //case asp_msgs::Ack_CompType_MSG_TYPE
				case asp_msgs::PlanerBeacon_CompType_MSG_TYPE :
				{
					auto beacon = std::static_pointer_cast<asp_msgs::PlanerBeacon>(msg);
					registerTeamMate(beacon->number());
					break;
				} //case asp_msgs::PlanerBeacon_CompType_MSG_TYPE
				default : break;
			} //switch ( msg_type )
			break;
		} //case 4711
		default : break;
	} //switch ( comp_id )
	recvTeam(endpoint, comp_id, msg_type, msg);
	return;
}

/**
 * @brief Registers the fact, that team mate X is (still) alive.
 * @param[in] number The number of the team mate.
 */
void
RefboxComm::registerTeamMate(const uint32_t number)
{
	MutexLocker locker(&MessageMutex);
	if ( TeamMates.count(number) == 0 )
	{
		//We detected a new mate!
		newTeamMate(number);
	} //if ( TeamMates.count(number) == 0 )
	TeamMates[number] = Now;
	return;
}

/**
 * @brief Checks if we have all the acks we need.
 * @param[in] message The message to be checked.
 * @return If we have all acks.
 */
bool
RefboxComm::checkAcks(const MessageStruct& message)
{
	switch ( message.NeedsAck )
	{
		case 0 : return true; //We shouldn't even be here.
		case 10 :
		{
			if ( message.ReceivedAcks.size() == TeamMates.size() )
			{
				return true;
			} //if ( message.ReceivedAcks.size() == TeamMates.size() )
			break;
		} //case 10 - All team mates have to ack.
		default :
		{
			assert(message.NeedsAck < 10);
			if ( message.ReceivedAcks.count(message.NeedsAck) )
			{
				return true;
			} //if ( message.ReceivedAcks.count(message.NeedsAck) )
		} //default - Only need ack from NeedsAck
	} //switch ( message.NeedsAck )

	return checkTeamMates() ? checkAcks(message) : false;
}

/**
 * @brief Checks if the team mates are considered alive.
 * @return If the number of team mates considered alive has been modified.
 */
bool
RefboxComm::checkTeamMates(void)
{
	constexpr auto timeout = std::chrono::seconds(10);
	const TimePoint limit = Now - timeout;

	MutexLocker locker(&MessageMutex);
	auto iter = TeamMates.begin();
	const auto end = TeamMates.end();
	bool modified = false;

	while ( iter != end )
	{
		if ( iter->second < limit )
		{
			const auto mate = iter->first;
			Log->log_info(LoggingComponent,
				"Team mate %d is considered dead, because we heard over %d seconds nothing from him.", mate,
				timeout.count());

			//We hope we find max one dead team member so we purge the acks immediatly.
			//When it is expected to find more than one dead robot save the numbers and remove them in one sweep.
			for ( MessageStruct& m : PeriodicMessages )
			{
				m.ReceivedAcks.erase(mate);
			} //for ( MessageStruct& m : PeriodicMessages )

			iter = TeamMates.erase(iter);
			modified = true;
			locker.unlock();
			deadTeamMate(mate);
		} //if ( iter->second < limit )
		else
		{
			++iter;
		} //else -> if ( iter->second < limit )
	} //while ( iter != end )
	return modified;
}

/**
 * @brief connects The peer to our error methods.
 * @param[in] peer the peer to connect.
 */
void
RefboxComm::setupPeer(ProtobufBroadcastPeer *peer) {
	peer->signal_recv_error().connect(boost::bind(&RefboxComm::recvError, this, _1, _2));
	peer->signal_send_error().connect(boost::bind(&RefboxComm::sendError, this, _1));
	return;
}

/**
 * @brief Opens the team channel.
 * @param[in] send The port used for sending.
 * @param[in] recv The port used for receiving.
 */
void
RefboxComm::openTeam(const unsigned short send, const unsigned short recv)
{
	MutexLocker locker(&ChannelMutex);
	try
	{
		const auto prefixLen = std::strlen(ConfigPrefix);
		char buffer[prefixLen + 20];
		const auto suffix = buffer + prefixLen;
		std::strcpy(buffer, ConfigPrefix);

		std::strcpy(suffix, "peer-address");
		const auto address = Config->get_string(buffer);
		std::strcpy(suffix, "crypto-key");
		const auto key     = Config->get_string(buffer);
		std::strcpy(suffix, "cipher");
		const auto cipher  = Config->get_string(buffer);

		TeamChannel = new ProtobufBroadcastPeer(address, send, recv, TeamRegister, key, cipher);
		setupPeer(TeamChannel);
		TeamChannel->signal_received().connect(boost::bind(&RefboxComm::recvTeamCommon, this, _1, _2, _3, _4));
		sendPeriodicMessages();
	} //try
	catch ( ... )
	{
		Log->log_warn(LoggingComponent, "No team refbox connection!");
	} //catch ( ... )
	return;
}

/**
 * @brief Sends an ack for a message and registers that it has seen this message.
 * @param[in] id The message id.
 * @return If this message is new, e.g. should be handled.
 */
bool
RefboxComm::ackMessageID(const uint32_t id)
{
	MutexLocker locker(&MessageMutex);
	auto ack = new asp_msgs::Ack;
	ack->set_id(id);
	ack->set_number(Number);
	PeriodicMessages.emplace_back(ack, StandardTimings::Ack, static_cast<unsigned short>(5));

	registerTeamMate(id % 10);

	const auto end = AckedMessages + ACK_PUFFER_SIZE;
	const auto pos = std::find(AckedMessages, end, id);
	if ( pos == end )
	{
		//The message was not yet acked.
		*NextAckPos = id;
		if ( ++NextAckPos == end )
		{
			NextAckPos = AckedMessages;
		} //if ( ++NextAckPos == end )
		return true;
	} //if ( pos == end )
	return false;
}

/**
 * @brief Sets the message id of a message.
 * @param[in, out] message The message to modify.
 * @param[in] needAck The value to set for the need ack field, cf. MessageStruct documentation.
 * @return The id.
 */
uint32_t
RefboxComm::setMessageID(MessageStruct& message, const uint32_t needAck) {
	assert(message.Counter == 0);
	message.NeedsAck = needAck;
	message.MessageID = NextMessageID + Number;
	NextMessageID += 10;
	return message.MessageID;
}

/**
 * @brief Will be called, when the game state has changed. The default implementation does nothing.
 * @param[in] newState The state in which the game is now.
 * @param[in] oldState The state in which the game was before.
 */
void
RefboxComm::gameStateChanged(const int /*newState*/, const int /*oldState*/)
{
	return;
}

/**
 * @brief Will be called, when the game phase has changed. The default implementation does nothing.
 * @param[in] newPhase The phase in which the game is now.
 * @param[in] oldPhase The phase in which the game was before.
 */
void
RefboxComm::gamePhaseChanged(const int /*newPhase*/, const int /*oldPhase*/)
{
	return;
}

/**
 * @brief Will be called everytime the beacon signal will be send. The default implementation does nothing.
 */
void
RefboxComm::updateBeacon(void)
{
	return;
}

/**
 * @brief Will be called, if a team mate is considered dead. The default implementation does nothing.
 * @param[in] mate The number of the team mate.
 */
void
RefboxComm::deadTeamMate(const uint32_t /*mate*/)
{
   return;
}

/**
 * @brief Will be called, if a new team mate is detected. The default implementation does nothing.
 * @param[in] mate The number of the team mate.
 */
void
RefboxComm::newTeamMate(const uint32_t /*mate*/)
{
	return;
}

/**
 * @brief Will be called, if we know which team we are. The default implementation does nothing.
 * @param[in] cyan If we are the cyan team.
 */
void
RefboxComm::setTeam(const bool /*cyan*/)
{
	return;
}

/**
 * @brief Will be called, if our team assignment is revoced. The default implementation does nothing.
 */
void
RefboxComm::unsetTeam(void)
{
	return;
}

/**
 * @brief Constructor.
 * @param[in] log The used logging facility.
 * @param[in] config The used configuration.
 *
 * Registers the common llsf_msgs and handled asp_msgs, additional (asp_)msgs have to be registered by the subclasses.
 */
RefboxComm::RefboxComm(Logger*& log, Configuration*& config) :
	ChannelMutex(Mutex::RECURSIVE), PublicChannel(nullptr), TeamChannel(nullptr), Number(0),
	ExplorationTime(0), GameState(new llsf_msgs::GameState),
	MessageMutex(Mutex::RECURSIVE), NextMessageID(0), NextAckPos(AckedMessages),
	Now(Clock::now()),
	PublicRegister(new MessageRegister()), TeamRegister(new MessageRegister()), Beacon(new llsf_msgs::BeaconSignal),
	BeaconStruct(nullptr),
	Log(log), Config(config), ConfigPrefix(nullptr), LoggingComponent("asp::RefboxComm::LoggingComponent")
{
	PublicRegister->add_message_type<llsf_msgs::BeaconSignal>();
	PublicRegister->add_message_type<llsf_msgs::ExplorationInfo>();
	PublicRegister->add_message_type<llsf_msgs::GameState>();
	PublicRegister->add_message_type<llsf_msgs::OrderInfo>();
	PublicRegister->add_message_type<llsf_msgs::RobotInfo>();
	PublicRegister->add_message_type<llsf_msgs::VersionInfo>();

	TeamRegister->add_message_type<llsf_msgs::BeaconSignal>();
	TeamRegister->add_message_type<llsf_msgs::ExplorationInfo>();
	TeamRegister->add_message_type<llsf_msgs::MachineInfo>();
	TeamRegister->add_message_type<llsf_msgs::MachineReport>();
	TeamRegister->add_message_type<llsf_msgs::MachineReportInfo>();
	TeamRegister->add_message_type<llsf_msgs::PrepareMachine>();
	TeamRegister->add_message_type<llsf_msgs::RingInfo>();
	TeamRegister->add_message_type<asp_msgs::Ack>();
	TeamRegister->add_message_type<asp_msgs::PlanerBeacon>();

	GameState->set_phase(llsf_msgs::GameState_Phase_SETUP);
	GameState->set_state(llsf_msgs::GameState_State_INIT);
	llsf_msgs::Time *time = GameState->mutable_game_time();
	time->set_sec(0);;
	time->set_nsec(0);
	return;
}

/**
 * @brief Destructor.
 */
RefboxComm::~RefboxComm()
{
	if ( publicOpen() )
	{
		//This closes a possible team connection as well.
		closePublic();
	}
	delete PublicRegister;
	delete TeamRegister;
	delete Beacon;
	return;
}

/**
 * @brief Inits the usage of the config. Call only after the config member is a valid pointer.
 */
void
RefboxComm::initRefboxComm(void)
{
	setConfigPrefix("/asp-agent/");
	return;
}

/**
 * @brief Checks if the public channel is open and if not tries to open it.
 * @return If the public channel is now open.
 */
bool
RefboxComm::checkPublic(void)
{
	if ( publicOpen() )
	{
		return true;
	} //if ( publicOpen() )
	openPublic();
	return publicOpen();
}

/**
 * @brief Checks if the public channel is open.
 * @return Wether the channel is open.
 */
bool
RefboxComm::publicOpen(void) const noexcept
{
	MutexLocker locker(&ChannelMutex);
	return PublicChannel;
}

/**
 * @brief Opens the public channel.
 */
void
RefboxComm::openPublic(void)
{
	MutexLocker locker(&ChannelMutex);
	assert(!publicOpen());
	try
	{
		const auto prefixLen = std::strlen(ConfigPrefix);
		char buffer[prefixLen + 20];
		const auto suffix = buffer + prefixLen;
		std::strcpy(buffer, ConfigPrefix);

		std::strcpy(suffix, "peer-address");
		const auto address  = Config->get_string(buffer);
		std::strcpy(suffix, "peer-send-port");
		const auto sendPort = Config->get_int(buffer);
		std::strcpy(suffix, "peer-recv-port");
		const auto recvPort = Config->get_int(buffer);

		PublicChannel = new ProtobufBroadcastPeer(address, sendPort, recvPort, PublicRegister);
		setupPeer(PublicChannel);
		PublicChannel->signal_received().connect(boost::bind(&RefboxComm::recvPublicCommon, this, _1, _2, _3, _4));
	} //try
	catch ( Exception& e )
	{
		Log->log_warn(LoggingComponent, "No Public Refbox connection, Fawkes Exception: %s", e.what_no_backtrace());
	} //catch ( Exception& e )
	catch ( std::exception& e )
	{
		Log->log_warn(LoggingComponent, "No Public Refbox connection, std::exception: %s", e.what());
	} //catch ( std::exception& e )
	catch ( ... )
	{
		Log->log_warn(LoggingComponent, "No Public Refbox connection!");
	} //catch ( ... )
	return;
}

/**
 * @brief Closes the public channel, assumes the channel is open. Closes the team channel if neccessary.
 */
void
RefboxComm::closePublic(void) noexcept
{
	if ( teamOpen() )
	{
		closeTeam();
	} //if ( teamOpen() )
	assert(publicOpen());
	MutexLocker locker(&ChannelMutex);
	delete PublicChannel;
	PublicChannel = nullptr;
	return;
}

/**
 * @brief Checks if the team channel is open.
 * @return Wether the channel is open.
 */
bool
RefboxComm::teamOpen(void) const noexcept
{
	return TeamChannel;
}

/**
 * @brief Closes the team channel. Assumes the channel is open.
 */
void
RefboxComm::closeTeam(void) noexcept
{
	assert(teamOpen());
	assert(publicOpen());
	MutexLocker locker(&ChannelMutex);
	delete TeamChannel;
	TeamChannel = nullptr;
	return;
}

/**
 * @brief Sets the confix prefix and extracts some basic information from the config. (Team name, number, etc.)
 * @param The new prefix.
 * @note This closes all existing channels and clears every message to send!
 */
void
RefboxComm::setConfigPrefix(const char *prefix)
{
	MutexLocker locker(&MessageMutex);
	PeriodicMessages.clear();
	std::fill(AckedMessages, AckedMessages + ACK_PUFFER_SIZE, 0);
	NextMessageID = 0;
	NextAckPos = AckedMessages;
	BeaconStruct = nullptr;

	if ( publicOpen() )
	{
		closePublic();
	} //if ( publicOpen() )

	ConfigPrefix = prefix;
	const auto prefixLen = std::strlen(ConfigPrefix);
	char buffer[prefixLen + 20];
	const auto suffix = buffer + prefixLen;
	std::strcpy(buffer, ConfigPrefix);

	std::strcpy(suffix, "robot-number");
	Number = Config->get_int(buffer);

	std::strcpy(suffix, "team-name");
	TeamName = Config->get_string(buffer);

	std::strcpy(suffix, "exploration-time");
	ExplorationTime = Config->get_uint(buffer);
	return;
}

/**
 * @brief Returns how many seconds game time have passed since the start.
 * @return The seconds.
 */
unsigned int
RefboxComm::gameTime(void) const
{
	MutexLocker locker(&GameStateMutex);
	return (GameState->phase() == llsf_msgs::GameState_Phase_PRODUCTION ? ExplorationTime : 0) +
		GameState->game_time().sec();
}

/**
 * @brief Returns how long the exploration phase is.
 * @return ExplorationTime
 */
unsigned int
RefboxComm::explorationTime(void) const noexcept
{
	return ExplorationTime;
}

/**
 * @brief Sets some values in the beacon, and adds it to the messages to be sent. Sets the BeaconStruct member.
 */
void
RefboxComm::activateBeacon(void)
{
	const auto prefixLen = std::strlen(ConfigPrefix);
	char buffer[prefixLen + 20];
	const auto suffix = buffer + prefixLen;
	std::strcpy(buffer, ConfigPrefix);

	std::strcpy(suffix, "robot-name");
	Beacon->set_peer_name(Config->get_string(buffer));

	Beacon->set_team_name(TeamName);
	Beacon->set_number(Number);
	Beacon->set_seq(0);

	MutexLocker locker(&MessageMutex);
	BeaconStruct = &*PeriodicMessages.emplace(PeriodicMessages.begin(), Beacon, StandardTimings::Beacon, false);
	return;
}

/**
 * @brief Sends all periodic messages, which have to be send.
 */
void
RefboxComm::sendPeriodicMessages(void)
{
	if ( !teamOpen() )
	{
		return;
	} //if ( !teamOpen() )

	Now = Clock::now();
	if ( BeaconStruct )
	{
		const auto now = Now.time_since_epoch();
		const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
		const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now - seconds);
		llsf_msgs::Time *time = Beacon->mutable_time();
		time->set_sec(seconds.count());
		time->set_nsec(nanoseconds.count());

		Beacon->set_seq(Beacon->seq() + 1);
		updateBeacon();
	} //if ( BeaconStruct )

	MutexLocker locker(&MessageMutex);
	auto iter = PeriodicMessages.begin();
	auto end = PeriodicMessages.end();

	while ( iter != end )
	{
		//Check delay
		if ( Now < iter->LastSend + iter->Delay )
		{
			//Must not be send, continue with next message.
			++iter;
			continue;
		} //if ( Now < iter->LastSend + iter->Delay )

		//Execute the function, if set
		if ( iter->Function )
		{
			iter->Function();
		} //if ( iter->Function )

		//Send message
		TeamChannel->send(*iter->Message);
		iter->LastSend = Now;

		//Check if message has a counter
		if ( iter->Counter )
		{
			//Decrease counter
			if ( --iter->Counter == 0 )
			{
				//When the counter hits 0 we either drop the message or switch to the old delay
				//Again, if there are multiple erases in one loop it becomes quadratic
				if ( iter->DelayAfterCounter == std::chrono::milliseconds::zero() )
				{
					iter = PeriodicMessages.erase(iter);
					end  = PeriodicMessages.end();
				} //if ( iter->DelayAfterCounter == std::chrono::milliseconds::zero() )
				else
				{
					iter->Delay = iter->DelayAfterCounter;
					iter->DelayAfterCounter = std::chrono::milliseconds::zero();
					++iter;
				} //else -> if ( iter->DelayAfterCounter == std::chrono::milliseconds::zero() )
			} //if ( --iter->Counter == 0 )
		} //if ( iter->Counter )
		else
		{
			//No counter, just continue with the next message.
			++iter;
		} //else -> if ( iter->Counter )
	} //while ( iter != end )
	return;
}

/**
 * @brief Returns the number of this team member.
 * @return The number.
 */
uint32_t
RefboxComm::number(void) const noexcept
{
	return Number;
}

}

}
