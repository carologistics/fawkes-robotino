/***************************************************************************
 * blackboard_processor.cpp - Processor for Bridge Requests Targeted for the
 *BlackBoard
 *
 *  Created: Mon April 11 2016
 *  Copyright  21016 Mostafa Gomaa
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

#include "blackboard_processor.h"

#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h" //TODO:: be moved when serializer is complete

#include <blackboard/blackboard.h>
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <interface/field_iterator.h>
#include <interface/interface.h>
#include <interface/interface_info.h>
#include <logging/logger.h>
#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <utils/time/time.h>

#include <cstdlib>
#include <cstring>

using namespace std;
using namespace fawkes;
using namespace rapidjson;

//=================================   Processor
//===================================

/** @class BridgeBlackBoardProcessor "blackboard_processor.h"
 * Derives from several Capability classes to provide those capabilities for to
 * BlackBoard Interface.
 *
 */

/** Constructor
 * @param prefix used to identify the BridgeProcessor
 * @param logger Fawkes logger
 * @param config Fawkes config
 * @param blackboard Fawkes blackboard
 * @param clock Fawkes clock
 */
BridgeBlackBoardProcessor::BridgeBlackBoardProcessor(std::string            prefix,
                                                     fawkes::Logger *       logger,
                                                     fawkes::Configuration *config,
                                                     fawkes::BlackBoard *   blackboard,
                                                     fawkes::Clock *        clock)
: BridgeProcessor(prefix)
{
	logger_     = logger;
	config_     = config;
	blackboard_ = blackboard;
	clock_      = clock;
	logger_->log_info("BlackBoardProcessor::", "Initialized!");
}

/** Destructor. */
BridgeBlackBoardProcessor::~BridgeBlackBoardProcessor()
{
	// for (ifi_ = interface_.begin(); ifi_ != interface_.end(); ++ifi_) {
	//   blackboard_->close(ifi_->second);
	// }
	// interface_.clear();
}

/** Perform a Subscription to a Blackboard Topic (handling a "subscribe" opcode)
 * checks if topic exists in the BlackBoard. If so, create a new
 * BlackBoardSubscription instance with the bb_interface information and the
 * request and return it.
 * @param prefixed_topic_name the session wishes to advertise
 * @param id provided in the JSON message of the rosbridge protocol
 * @param type provided in the JSON message of the rosbridge protocol
 * @param compression provided in the JSON message of the rosbridge protocol
 * @param throttle_rate provided in the JSON message of the rosbridge protocol
 * @param queue_length provided in the JSON message of the rosbridge protocol
 * @param fragment_size provided in the JSON message of the rosbridge protocol
 * @param session The session that issued the request and that expects a publish
 * on change of topic data
 * @return BlackboardSubscription
 */
std::shared_ptr<Subscription>
BridgeBlackBoardProcessor::subscribe(std::string                 prefixed_topic_name,
                                     std::string                 id,
                                     std::string                 type,
                                     std::string                 compression,
                                     unsigned int                throttle_rate,
                                     unsigned int                queue_length,
                                     unsigned int                fragment_size,
                                     std::shared_ptr<WebSession> session)
{
	// TODO:: what happens if u subscribed to an interface before its posted.
	// For now we throw an exception assuming something is wrong..but it could be
	// that it we're just not there yet

	// Extract prefix from the name
	std::string topic_name = "";

	std::size_t pos = prefixed_topic_name.find(prefix_, 0);
	if (pos != std::string::npos && pos <= 1) {
		topic_name = prefixed_topic_name.erase(
		  0, prefix_.length() + pos + 1); //+1 accounts for the leading '/' before the topic name
	}

	if (topic_name == "") {
		// this should not happen
		throw fawkes::SyntaxErrorException(
		  "BlackBoardProcessor: Topic Name '%s' does not contian prefix of %s "
		  "processor ",
		  prefixed_topic_name.c_str(),
		  prefix_.c_str());
	}

	// TODO::take care of the differnt ways the topic is spelled

	// Extracet the BlackBoard Interface Type and Id
	pos                 = topic_name.find("::", 0);
	std::string if_id   = topic_name;
	std::string if_type = topic_name;

	if (pos != std::string::npos) {
		if_type.erase(pos, topic_name.length());
		if_id.erase(0, pos + 2);
		// logger_->log_info("BlackboardProcessor: if_type",if_type.c_str());
		// logger_->log_info("BlackboardProcessor: if_id",if_id.c_str());
	} else {
		throw fawkes::SyntaxErrorException("BlackBoardProcessor: Wrong 'Topic Name' format!");
	}

	// Look for the topic in the BlackBoard intefaces
	bool found = false;

	InterfaceInfoList *iil = blackboard_->list_all();
	for (InterfaceInfoList::iterator i = iil->begin(); i != iil->end(); ++i) {
		if (if_type.compare(i->type()) == 0 && if_id.compare(i->id()) == 0) {
			found = true;
			break;
		}
	}
	delete iil;

	if (!found) {
		throw fawkes::UnknownTypeException("BlackboardProcessor: Interface '%s' was not found!",
		                                   topic_name.c_str());
	}

	std::shared_ptr<BlackBoardSubscription> new_subscirption;
	// std::shared_ptr <Subscription> new_subscirption;

	// Open the interface and create a DORMANT subscription instance with it
	try {
		new_subscirption =
		  std::make_shared<BlackBoardSubscription>(topic_name,
		                                           prefix_,
		                                           logger_,
		                                           clock_,
		                                           blackboard_,
		                                           blackboard_->open_for_reading(if_type.c_str(),
		                                                                         if_id.c_str()));

		// new_subscirption = std::make_shared <Subscription>(topic_name
		//                                                           , prefix_
		//                                                           , clock_ );

	} catch (fawkes::Exception &e) {
		logger_->log_info("BlackBoardProcessor:",
		                  "Failed to open subscribe to '%s': %s\n",
		                  topic_name.c_str(),
		                  e.what());
		throw e;
	}

	logger_->log_info("BlackboardProcessor:",
	                  "Interface '%s' Succefully Opened!",
	                  topic_name.c_str());

	// TODO:: add_request should be removed and the request is add on Subscription
	// during construction/ OR moved to the SubscriptionCapabilty manager maybe.
	new_subscirption->add_request(
	  id, compression, throttle_rate, queue_length, fragment_size, session);

	return new_subscirption;
}

void
BridgeBlackBoardProcessor::unsubscribe(std::string                   id,
                                       std::shared_ptr<Subscription> subscription,
                                       std::shared_ptr<WebSession>   session)
{
	std::shared_ptr<BlackBoardSubscription> bb_subscription =
	  std::static_pointer_cast<BlackBoardSubscription>(subscription);

	bb_subscription->remove_request(id, session);
}

std::shared_ptr<Advertisment>
BridgeBlackBoardProcessor::advertise(std::string                 topic_name,
                                     std::string                 id,
                                     std::string                 type,
                                     std::shared_ptr<WebSession> session)
{
	// To Be Implemented
	std::shared_ptr<Advertisment> advertisment = std::make_shared<Advertisment>(topic_name, prefix_);
	advertisment->add_request(id, session);
	return advertisment;
}

void
BridgeBlackBoardProcessor::unadvertise(std::string                   id,
                                       std::shared_ptr<Advertisment> advertisment,
                                       std::shared_ptr<WebSession>   session)
{
	// To Be Implemented
}

/** Handle opcode "publish", updating the topic data with the data in the json
 * msg. This is the implementation of the publish operation of the
 * AdvertismentCapability. It will be called when a session wants to publish a
 * topic (writing the data to the bb_interface). Usually this comes after an
 * "advertise" message allowing the session to change the topic data.
 * @param  id specified in rosbirdge protocol
 * @param  latch specified in rosbirdge protocol
 * @param  msg_in_json The topic data to update
 * @param advertisment The Advertisement instance created to update this topic
 * @param session The session that will do the publishing
 */
void
BridgeBlackBoardProcessor::publish(
  std::string id,
  bool        latch,
  std::string msg_in_json // TODO:: figure out a clever way to keep track of
                          // msgs types and content without the need to have
                          // the info before hands
  ,
  std::shared_ptr<Advertisment> advertisment,
  std::shared_ptr<WebSession>   session)
{
	// To Be Implemented
}

//====================================  Subscription
//===========================================

/** @class BlackBoardSubscription "blackboard_processor.h"
 * Derives from Subscription class and extends its behaviour to enable
 * publishing of BlackBoard topics. Derives from BlackBoardInterfaceListener to
 * listen to changes made on a topic and publishes when needed.
 */

/** Constructor
 * @param topic_name Full topic name, Including the "blackboard/" prefix
 * @param processor_prefix The Processor's unique
 *@param logger Fawkes logger
 * @param clock Fawkes clock
 * @param blackboard blackboard where the topic lives
 * @param interface interface to access the topic from blackboard
 */
BlackBoardSubscription::BlackBoardSubscription(std::string         topic_name,
                                               std::string         processor_prefix,
                                               fawkes::Logger *    logger,
                                               fawkes::Clock *     clock,
                                               fawkes::BlackBoard *blackboard,
                                               fawkes::Interface * interface)
: Subscription(topic_name, processor_prefix, logger, clock),
  BlackBoardInterfaceListener("WebToolsBridgeListener"),
  blackboard_(blackboard),
  interface_(interface)
{
}

/** Destructor */
BlackBoardSubscription::~BlackBoardSubscription()
{
	// Not Needed
	// if (interface_ != NULL )
	// {
	// delete this->interface_;
	// }
}

/** @return the internal interface ptr */
fawkes::Interface *
BlackBoardSubscription::get_interface_ptr()
{
	return interface_;
}

void
BlackBoardSubscription::activate_impl()
{
	bbil_add_data_interface(interface_);
	blackboard_->register_listener(this);
}

void
BlackBoardSubscription::deactivate_impl()
{
	bbil_remove_data_interface(interface_);
	blackboard_->unregister_listener(this);
}

void
BlackBoardSubscription::finalize_impl()
{
	blackboard_->close(interface_);
}

/**Creates a "publish" rosbridge protocol JSON Message
 * serializes the blackboard topic data into the "msg:" field of
 * the rosbridge protocol, "publish" opcode, JSON message
 * @param op name of the operation according to rosbridge protocol
 * @param topic_name name of the topic to be serialized
 * @param id
 * @return the serialized JSON string
 */
std::string
BlackBoardSubscription::serialize(std::string op, std::string topic_name, std::string id)
{
	std::string prefixed_topic_name = processor_prefix_ + "/" + topic_name;

	// Default 'publish' header
	StringBuffer         s;
	Writer<StringBuffer> writer(s);
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(), (SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(), (SizeType)id.length());

	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.String("msg");

	// Start of Data Json creation
	writer.StartObject();

	// Data filling from interface
	for (InterfaceFieldIterator fi = interface_->fields(); fi != interface_->fields_end(); ++fi) {
		std::string fieldName = fi.get_name();

		writer.String(fieldName.c_str(), (SizeType)fieldName.length());

		std::string fieldType = fi.get_typename();

		if (fi.get_length() > 1 && fieldType != "string") {
			writer.StartArray();

			if (fieldType == "bool") {
				bool *arr = fi.get_bools();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Bool(arr[i]);
			}

			else if (fieldType == "double") {
				double *arr = fi.get_doubles();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Double(arr[i]);
			}

			else if (fieldType == "float") {
				float *arr = fi.get_floats();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Double(arr[i]);
			}

			else if (fieldType == "uint64") {
				uint64_t *arr = fi.get_uint64s();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Uint64(arr[i]);
			}

			else if (fieldType == "uint32") {
				uint32_t *arr = fi.get_uint32s();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Uint(arr[i]);
			}

			else if (fieldType == "uint16") {
				uint16_t *arr = fi.get_uint16s();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Uint(arr[i]);
			}

			else if (fieldType == "uint8") {
				uint8_t *arr = fi.get_uint8s();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Uint(arr[i]);
			}

			else if (fieldType == "int64") {
				uint64_t *arr = fi.get_uint64s();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Int64(arr[i]);
			}

			else if (fieldType == "int32") {
				int32_t *arr = fi.get_int32s();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Int(arr[i]);
			}

			else if (fieldType == "int16") {
				int16_t *arr = fi.get_int16s();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Int(arr[i]);
			}

			else if (fieldType == "int8") {
				int8_t *arr = fi.get_int8s();
				for (unsigned i = 0; i < fi.get_length(); i++)
					writer.Int(arr[i]);
			}

			writer.EndArray();

		} else {
			if (fieldType == "bool")
				writer.Bool(fi.get_bool());

			else if (fieldType == "string")
				writer.String(fi.get_string());

			else if (fieldType == "double")
				writer.Double(fi.get_double());

			else if (fieldType == "float")
				writer.Double(fi.get_float());

			else if (fieldType == "uint32")
				writer.Uint(fi.get_uint32());

			else if (fieldType == "int32")
				writer.Int(fi.get_int32());

			else if (fieldType == "int8")
				writer.Int(fi.get_int8());

			else if (fieldType == "uint8")
				writer.Uint(fi.get_uint8());

			else if (fieldType == "int16")
				writer.Int(fi.get_int16());

			else if (fieldType == "uint16")
				writer.Uint(fi.get_uint16());

			else if (fieldType == "int64")
				writer.Int64(fi.get_int64());

			else if (fieldType == "uint64")
				writer.Uint64(fi.get_uint64());

			else
				writer.String(fi.get_value_string());
		}

		//  else if (fieldType== "byte");
		//  else if (fieldType== "unknown")
		//  else if (fieldType== "_info->enumtype") find out where is this coming
		//  from

		// writer.Null();  find what null means in blackboard...if it exists
	}

	writer.EndObject(); // End of data json

	writer.EndObject(); // End of complete Json_msg

	return s.GetString();
}

/**  Register To Listen To BlackBoard Interface's Change Events
 * Trigger a publish whenever a change happens On the BlackBoard topic
 * @param interface The interface to listen too
 */
void
BlackBoardSubscription::bb_interface_data_refreshed(fawkes::Interface *interface) throw()
{
	if (!is_active()) {
		// this should not happen. listener should be deregistered in
		// deactivate_impl()
		return;
	}

	interface_->read();

	publish();
}