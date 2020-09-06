/***************************************************************************
 *  advertisment_capability.h - Advertisment Capabiliy Abstract Class
 *  Created: 2016
 *  Copyright  2016 Mostafa Gomaa
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

#ifndef __PLUGINS_ADVERTIS_CAPABILITY_H_
#define __PLUGINS_ADVERTIS_CAPABILITY_H_

#include "callable.h"
#include "event_emitter.h"
#include "event_type.h"

#include <list>
#include <map>
#include <memory>
#include <string>

namespace fawkes {
class Mutex;
}

class Advertisment;
class WebSession;
class EventEmitter;

/** @class AdvertismentCapability
 * Abstract class, providing session the ability to un/register as a publisher
 * (un/advertised) for a certain topic.
 * The semantics of what an advertisement means, is up to the processor that
 * implements this capability (because it is specific to the domain).
 *
 * @author Mostafa Gomaa
 */
class AdvertismentCapability
{
public:
	/** To be implemented by the bridge processor for domain specific semantics
   * @param topic_name the session wishes to advertise
   * @param id specified in the JSON message
   * @param type  specified in the  JSON message
   * @param session the session requesting the advertise
   * @return ptr to a newly created dormant advertisement instance
   */
	virtual std::shared_ptr<Advertisment> advertise(std::string                 topic_name,
	                                                std::string                 id,
	                                                std::string                 type,
	                                                std::shared_ptr<WebSession> session) = 0;

	/** To be implemented by the bridge processor for domain specific semantics
   * @param id included in the JSON message
   * @param advertisment ptr to the instance that we need to unadvertised
   * @param session the session initiating requesting
   */
	virtual void unadvertise(std::string                   id,
	                         std::shared_ptr<Advertisment> advertisment,
	                         std::shared_ptr<WebSession>   session) = 0;

	/** To be implemented by the bridge processor for domain specific semantics
   * @param id included in the JSON message
   * @param latch included in the JSON message
   * @param msg_in_json to be published for that topic
   * @param advertisment ptr to the instance that we need to publish on
   * @param session the session initiating requesting
   */
	virtual void publish(std::string id,
	                     bool        latch,
	                     std::string msg_in_json // TODO:: figure out a clever way to keep
	                                             // track of msgs types and content without the
	                                             // need to have the info before hands
	                     ,
	                     std::shared_ptr<Advertisment> advertisment,
	                     std::shared_ptr<WebSession>   session) = 0;
};

class Advertisment : public Callable,
                     public EventEmitter,
                     public std::enable_shared_from_this<Advertisment>
{
public:
	Advertisment(std::string topic_name, std::string processor_prefix);

	virtual ~Advertisment();

	void        finalize();
	void        activate();
	void        deactivate();
	bool        is_active();
	bool        empty();
	std::string get_topic_name();
	std::string get_processor_prefix();
	void        subsume(std::shared_ptr<Advertisment> another_advertisment);

	void add_request(std::string id, std::shared_ptr<WebSession> session);
	void remove_request(std::string id, std::shared_ptr<WebSession> session);

	void callback(EventType event_type, std::shared_ptr<EventEmitter> handler);

	void emitt_event(EventType event_type);

protected:
	virtual void finalize_impl();   /**< Implicitly called from finalize() */
	virtual void activate_impl();   /**< Implicitly called from activate() */
	virtual void deactivate_impl(); /**< implicitly called from deactivate() */

	fawkes::Mutex *__mutex; /**< needed to lock the eternal registry */

protected:
	/** @brief the possible status of an instance. A DORMANT instance only keeps
   * the data.*/
	enum Status { ACTIVE, DORMANT };

	/** @brief a single Request mad by a session for advertisement. A session
   * can make multiple requests with different request parameters. */
	struct Request
	{
		Request() : id("")
		{
		}
		~Request(){};
		std::string id; /**< id given to request. Unique across a single
                       advertisement instance*/
	};

	Status      active_status_;    /**< Tracks the status of the instance */
	std::string topic_name_;       /**< Topic name this instance is maintaining*/
	std::string processor_prefix_; /**< processor specific prefix included in that
                                    topic name*/

	bool finalized; /**< set to true if it was Object was finilazed berfore */

	/// Contains mapping of session to request it made
	std::map<std::shared_ptr<WebSession>, std::list<Request>> advertisments_;
	/// Iterator for advertisement_ map
	std::map<std::shared_ptr<WebSession>, std::list<Request>>::iterator it_advertisments_;
	/// Iterator for requests list
	std::list<Request>::iterator it_requests_;
};

#endif
