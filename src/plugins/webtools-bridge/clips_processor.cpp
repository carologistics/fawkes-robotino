
/***************************************************************************
 * clips_processor.cpp - Processor for Bridge Requests Targeted for Clips Facts
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

#include "clips_processor.h"

#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h" //TODO:: be moved when serializer is complete
#include "serializer.h"

#include <clips/clips.h> //am not sure i even need this ..lets see
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>
#include <plugins/clips/aspect/clips_env_manager.h>

#include <clipsmm.h>
#include <iostream>
#include <map>
#include <thread>
#include <unistd.h>

using namespace fawkes;
using namespace rapidjson;

/** @class ClipsSubscription "clips_processor.h"
 * Implements "Susbscription" object with clips facts in mind as a topic.
 * It holds objects and environments necessary to access the clips fact when
 * publishing, and implements Serialize(), that prescribes how to get a CLIPS
 * fact's data and serializes in a publish JSON message ready to published to
 * the web client via WebSession.
 */

/** Consturctor
 * @param topic_name the full name of topic that will be subscribed to
 * (including the "clips/" prefix)
 * @param processor_prefix the "clips/" prefix
 * @param logger Fawkes logger
 * @param clock Fawkes clock
 * @param clips the clips environment, necessary access the fact data
 * @param clips_env_mgr the clips environment manager, necessary access the fact
 * data
 */
ClipsSubscription::ClipsSubscription(std::string                               topic_name,
                                     std::string                               processor_prefix,
                                     fawkes::Logger *                          logger,
                                     fawkes::Clock *                           clock,
                                     fawkes::LockPtr<CLIPS::Environment> &     clips,
                                     fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips_env_mgr)
: Subscription(topic_name, processor_prefix, logger, clock),
  clips_(clips),
  clips_env_mgr_(clips_env_mgr)
{
}

ClipsSubscription::~ClipsSubscription()
{
}

void
ClipsSubscription::activate_impl()
{
	// std::thread t(&ClipsSubscription::publish_loop, this);
	// t.detach();
}

void
ClipsSubscription::deactivate_impl()
{
}

void
ClipsSubscription::finalize_impl()
{
}

/* Creates the JSON message to be Published
 * This method is derived from Subscription, it prescribes how to get a
 * CLIPS fact's data and serialzes in a JSON message with a rosbridge "publish"
 * opcode ready to published. This is called from by the publish() method of the
 * Subscription whenever the subscription wants to publish.
 * @param op name of the operation according to rosbridge protocol
 * @param topic_name name of the topic to be serialized
 * @param id
 * @return the serialized json string
 */
std::string
ClipsSubscription::serialize(std::string op, std::string topic_name, std::string id)
{
	std::string prefixed_topic_name = processor_prefix_ + "/" + topic_name;
	std::string tmpl_name           = topic_name;
	std::string env_name_           = "agent";

	std::map<std::string, LockPtr<CLIPS::Environment>> envs = clips_env_mgr_->environments();
	if (envs.find(env_name_) == envs.end()) {
		if (envs.size() == 1) { // if there is only one just select it
			env_name_ = envs.begin()->first;
		} else {
			throw fawkes::UnknownTypeException("ClipsProcessor: Environment '%s' was not found!",
			                                   env_name_.c_str());
			// TODO:say what was wrong no envs Vs Wrong name
		}
	}
	clips_ = envs[env_name_];
	MutexLocker          lock(clips_.objmutex_ptr());
	CLIPS::Fact::pointer fact = clips_->get_facts(); // the first fact in the env

	// does fact exist
	bool fact_found = false;
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name().compare(tmpl_name) == 0) {
			fact_found = true;
			// std::cout << "found" << std::endl;
			break;
		}
		fact = fact->next();
	}

	if (!fact_found) {
		// logger_->log_info("ClipsProcessor", "couldn't find fact with template
		// name %s for now", tmpl_name.c_str());
		return "";
	} else {
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
		// Serilaize the all matching facts
		/*
    *Non-ordered fact:
    {fact: [
                 { slot1: [ value1, vlaue2] , slot2:[] } , //a mathcing fact
                 { slot1: [ value1, vlaue2] , slot2:[] } , //another mathcing
    fact
            ]
    }

    *ordered fact
    {fact: [
                 { fleids: [ valu1, vlaue2 , value3 ] } , //a mathcing fact
                 { fleids: [] } ,                         //another mathcing
    fact
            ]
    }

    {fact:[]} //a predicate
    */
		writer.StartObject(); // start of msg json
		std::string name = fact->get_template()->name();
		writer.String(name.c_str(), (SizeType)name.length()); // factName: as the
		                                                      // Key

		writer.StartArray(); // start of fact jsons array
		CLIPS::Fact::pointer fact = clips_->get_facts();
		while (fact) {
			CLIPS::Template::pointer tmpl = fact->get_template();
			if (tmpl->name().compare(tmpl_name) == 0) {
				std::vector<std::string> slot_names = fact->slot_names();
				bool ordered_fact   = slot_names.size() == 1 && slot_names[0] == "implied" ? true : false;
				bool predicate_fact = ordered_fact && fact->slot_value("").size() < 1 ? true : false;

				// std::cout <<slot_names[0]<<std::endl;

				// A Non-ordered Fact Serialization
				if (!ordered_fact) {
					writer.StartObject(); // start of 'A' fact json
					for (std::vector<std::string>::iterator it = slot_names.begin();
					     slot_names.size() > 0 && it != slot_names.end();
					     it++) {
						writer.String(it->c_str(),
						              (SizeType)it->length()); // slot_name as key
						CLIPS::Values values = fact->slot_value(*it);
						writer.StartArray(); // array of slot values
						for (CLIPS::Values::iterator it2 = values.begin();
						     values.size() > 0 && it2 != values.end();
						     it2++) {
							switch (it2->type()) {
							case CLIPS::TYPE_INTEGER: writer.Int64(it2->as_integer()); break;
							case CLIPS::TYPE_FLOAT: writer.Double(it2->as_float()); break;
							default: writer.String(it2->as_string().c_str(), (SizeType)it2->as_string().length());
							}
						}
						writer.EndArray(); // end slot values array
					}
					writer.EndObject(); // end of 'one' fact json
				}

				// An Ordered Fact Serialization
				else {
					if (!predicate_fact) {
						writer.StartObject();    // start of 'A' fact json
						writer.String("fields"); // key for fields array
						writer.StartArray();     // start of fields array
						CLIPS::Values values = fact->slot_value("");
						for (CLIPS::Values::iterator it = values.begin();
						     values.size() != 0 && it != values.end();
						     it++) {
							switch (it->type()) {
							case CLIPS::TYPE_INTEGER: writer.Int64(it->as_integer()); break;
							case CLIPS::TYPE_FLOAT: writer.Double(it->as_float()); break;
							default:
								writer.String(it->as_string().c_str(), (SizeType)it->as_string().length());
								break;
							}
						}
						writer.EndArray();  // end fields array
						writer.EndObject(); // end 'A' fact json
					}
				}
			}
			fact = fact->next();
		}

		writer.EndArray();  // end of fact jsons array
		writer.EndObject(); // End of msg json

		writer.EndObject(); // End of complete Json_msg

		// std::cout << s.GetString()<<std::endl;
		return s.GetString();
	}
}

// void
// ClipsSubscription::publish_loop()
// {
//   while(true)
//   {

//     publish();
//   //  sleep(100);
//   }

// }

//=================================   Processor
//===================================

/** @class ClipsProcessor "clips_processor.h"
 * Derives from different Capability classes and provides those Capabilities for
 * Clips FActs
 */

/** Constructor
 * @param prefix The Processor's unique
 * @param logger Fawkes logger
 * @param config Fawkes config
 * @param clock Fawkes clock
 * @param clips_env_mgr the clips enviorment manager
 */
ClipsProcessor::ClipsProcessor(std::string                               prefix,
                               fawkes::Logger *                          logger,
                               fawkes::Configuration *                   config,
                               fawkes::Clock *                           clock,
                               fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips_env_mgr)
: BridgeProcessor(prefix), env_name_("agent") // TODO: get from configs
{
	clips_env_mgr_ = clips_env_mgr;
	logger_        = logger;
	config_        = config;
	clock_         = clock;

	logger_->log_info("ClipsProcessor::", "Initialized!");

	// TODO set the env_name
}

/** Destructor. */
ClipsProcessor::~ClipsProcessor()
{
}

void
ClipsProcessor::init()
{
	if (!initialized_) {
		std::map<std::string, LockPtr<CLIPS::Environment>> envs = clips_env_mgr_->environments();

		if (envs.find(env_name_) == envs.end()) {
			if (envs.size() == 1) { // if there is only one just select it
				env_name_ = envs.begin()->first;
			} else {
				throw fawkes::UnknownTypeException("ClipsProcessor: Environment '%s' was not found!",
				                                   env_name_.c_str());
				// say what was wrong no envs Vs Wrong name
			}
		}

		clips_ = envs[env_name_];

		BridgeProcessor::init();
	}
}

std::shared_ptr<Subscription>
ClipsProcessor::subscribe(std::string                 prefixed_topic_name,
                          std::string                 id,
                          std::string                 type,
                          std::string                 compression,
                          unsigned int                throttle_rate,
                          unsigned int                queue_length,
                          unsigned int                fragment_size,
                          std::shared_ptr<WebSession> web_session)
{
	std::string tmpl_name = ""; // get it from the prefixed topic name

	std::size_t pos = prefixed_topic_name.find(prefix_, 0);
	if (pos != std::string::npos && pos <= 1) {
		tmpl_name = prefixed_topic_name.substr(
		  pos + prefix_.length() + 1); //+1 accounts for the leading '/' before the topic name
	}

	std::map<std::string, LockPtr<CLIPS::Environment>> envs = clips_env_mgr_->environments();
	if (envs.find(env_name_) == envs.end()) {
		if (envs.size() == 1) { // if there is only one just select it
			env_name_ = envs.begin()->first;
		} else {
			throw fawkes::UnknownTypeException("ClipsProcessor: Environment '%s' was not found!",
			                                   env_name_.c_str());
			// say what was wrong no envs Vs Wrong name
		}
	}
	clips_ = envs[env_name_];
	MutexLocker lock(clips_.objmutex_ptr());

	CLIPS::Fact::pointer fact       = clips_->get_facts(); // intialize it with the first fact
	bool                 fact_found = false;
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name().compare(tmpl_name) == 0) {
			fact_found = true;
			break;
			// for now, its just enough to know that there is a fact with this
			// topic_name to make a subscription Realisticly, u need to know the
			// constraint and fit to it more over if nothing fits now or no fact with
			// tmp_name found..Does not mean that this subscription is not valid.
			// Could mean that the fact has not been asserted yet and that we need to
			// watch for it So basically i think there is no invalide subscriptions
			// with clips..Just ones that u need to watch for.
		}
		fact = fact->next();
	}

	if (!fact_found)
		logger_->log_info("ClipsProcessor",
		                  "couldn't find fact with template name %s for now",
		                  tmpl_name.c_str());

	// for now, always make a subscription intstance for the subscription request
	// (even if no fact found)
	std::shared_ptr<ClipsSubscription> new_subscirption;

	// a DORMANT subscription instance with it
	try {
		new_subscirption = std::make_shared<ClipsSubscription>(
		  tmpl_name, prefix_, logger_, clock_, clips_, clips_env_mgr_);

	} catch (fawkes::Exception &e) {
		logger_->log_info(
		  "ClipsProcessor:", "Failed to subscribe to '%s': %s\n", tmpl_name.c_str(), e.what());
		throw e;
	}

	new_subscirption->add_request(
	  id, compression, throttle_rate, queue_length, fragment_size, web_session);

	logger_->log_info("ClipsProcessor", "DONE");

	return new_subscirption;
}

void
ClipsProcessor::unsubscribe(std::string                   id,
                            std::shared_ptr<Subscription> subscription,
                            std::shared_ptr<WebSession>   session)
{
}
