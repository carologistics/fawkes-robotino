/***************************************************************************
 *  advertisment_capability.cpp - Advertisment Capabiliy Abstract Class
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

#include "advertisment_capability.h"

#include "web_session.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/time.h>

#include <list>
#include <map>
#include <memory>

using namespace fawkes;

/** @class Advertisment "advertisment_capability.h"
 * A Advertisement instance tracks advertisement requests made from various
 * sessions to 'a' topic. For a single topic name, It is responsible for book
 * keeping and maintaining the sessions that wishes to advertise on that
 * topic_name.
 *
 * @author Mostafa Gomaa
 */

/** Constructor.
 * @param topic_name Full name of the topic, prefixed with the BridgeProcessor
 * it targets
 * @param prefix the prefix that identifies the BridgeProcessor
 */
Advertisment::Advertisment(std::string topic_name, std::string prefix)
: active_status_(DORMANT), topic_name_(topic_name), processor_prefix_(prefix), finalized(false)
{
	__mutex = new fawkes::Mutex();
}

/** Disstructor. */
Advertisment::~Advertisment()
{
	advertisments_.clear();
	delete __mutex;
}

/** Finalize an advertisment before terminating the instance. */
void
Advertisment::finalize()
{
	MutexLocker ml(__mutex);

	if (!finalized) {
		// If still active deactivate
		if (is_active()) {
			deactivate();
		}
		// call the extended version
		finalize_impl();

		// Deregister from sessions and remove them
		if (!empty()) {
			for (it_advertisments_ = advertisments_.begin(); it_advertisments_ != advertisments_.end();) {
				it_advertisments_->first->unregister_callback(EventType::TERMINATE, shared_from_this());
				it_advertisments_->second.clear();
				advertisments_.erase(it_advertisments_++);
			}
		}

		finalized = true;
	}
}

/** Activate the Advertisement instance
 * It Changes the status of the instance to ACTIVE, enabling the publish to be
 * called. When an advertisement is created, it is initially DORMANT. The method
 * will call Activate_impl() by default to execute any extended behaviours.
 */
void
Advertisment::activate()
{
	if (!is_active()) {
		activate_impl();
		active_status_ = ACTIVE;
	}
}

/** Change Status into DORMANT
 * When a Subscription is DORMANT it can not publish and it could be subsumed by
 * * another ACTIVE subscription.
 * The method will call deactivate_impl() by default to execute any extended
 * behaviours.
 */
void
Advertisment::deactivate()
{
	if (is_active()) {
		deactivate_impl();
		active_status_ = DORMANT;
	}
}

/** @return True when its an ACTIVE instance */
bool
Advertisment::is_active()
{
	return (active_status_ == ACTIVE);
}

/** @return True when this instance has no session registered for topic
 * advertisement*/
bool
Advertisment::empty()
{
	return advertisments_.empty();
}

/** @return topic name maintained by this advertisement instance */
std::string
Advertisment::get_topic_name()
{
	return topic_name_;
}

/** @return the processor prefix of that topic */
std::string
Advertisment::get_processor_prefix()
{
	return processor_prefix_;
}

/** Finalize extended behaviour
 * This method will be called by default by Finalize()
 * Extend this if you wish to add any operations upon finalizing of the your
 * Subscription. After this the your instance should be ready to be terminated.
 */
void
Advertisment::finalize_impl()
{
	// Override to extend behaviour
}

/** Activate extended behaviour
 * This method will be called by default from Activate()
 * Activate_impl() will be called right before the instance becomes active.
 * Exemplary procedures to be done in Activate_impl(), are registering for
 * events listener relevant to topics changes, Or prescribing when the publish
 * should be triggered.
 */
void
Advertisment::activate_impl()
{
	// Override to extend behaviour
}

/** Deactivate extended behaviour
 * This method will be called by default from dectivate()
 * any operation done to make the instance active "could be" undone here.
 */
void
Advertisment::deactivate_impl()
{
	// Override to extend behaviour
}

/** Subsumes a DORMANT Advertisement instance into an ACTIVE one.
 * This is usually called when there is more than one Advertisement instance for
 * the same topic. The owning instance must be Active and the instance to be
 * subsumed must to be Dormant ie, ActiveInstance.Subsume(DormantInstance).
 * After the call, the dormant instance could be safly deleted.
 * @param dormant_advertisment instance to be subsumed
 */
void
Advertisment::subsume(std::shared_ptr<Advertisment> dormant_advertisment)
{
	if (topic_name_ != dormant_advertisment->get_topic_name()) {
		// throw exception that they dont belong to the same topic and cant be
		// merged
		return;
	}

	if (!is_active()) {
		// throw exceptoin that they dont belong to the same topic and cant be
		// merged
		return;
	}

	if (dormant_advertisment->is_active()) {
		// throw exception that they dont belong to the same topic and cant be
		// merged
		return;
	}

	for (std::map<std::shared_ptr<WebSession>, std::list<Request>>::iterator it_advertisments =
	       dormant_advertisment->advertisments_.begin();
	     it_advertisments != dormant_advertisment->advertisments_.end();
	     it_advertisments++) {
		for (std::list<Request>::iterator it_requests = it_advertisments->second.begin();
		     it_requests != it_advertisments->second.end();
		     it_requests++) {
			add_request(it_requests->id, it_advertisments->first);
		}
	}
	dormant_advertisment->finalize();
}

/** Process a request by add an entry for the request mapped to it's session.
 * Usually called by advertise() at the capability manager.
 * @param id of the request generated by the capability manager.
 * @param session the session requesting to advertise the topic maintained by
 * this instance.
 */
void
Advertisment::add_request(std::string id, std::shared_ptr<WebSession> session)
{
	Request request;
	// CHANGE:this matches for the pointer not the object
	MutexLocker ml(__mutex);

	it_advertisments_ = advertisments_.find(session);

	// if it is a new session, register my terminate_handler for the session's
	// callbacks
	if (it_advertisments_ == advertisments_.end()) {
		session->register_callback(EventType::TERMINATE, shared_from_this());
	}

	// //if there was older requests for this session,  point to the same
	// last_published_time if ( it_advertisments_ != advertisments_.end() &&
	// !(advertisments_[session].empty()) )
	// {
	// 	if(advertisments_[session].find(id) != advertisments_[session].end())
	// 	{
	// 		//throw exception..That id already exists for that client on
	// that topic
	// 	}
	// }

	request.id = id;

	advertisments_[session].push_back(request);
}

/** Remove a request upon an unadvertise operation
 * Process a request to unadvertise by removing the corresponding entry.
 * Usually called by unadvertise() at the capability manager.
 * @param advertisment_id of the request to remove
 * @param session responsible for the request.
 *
 */
void
Advertisment::remove_request(std::string advertisment_id, std::shared_ptr<WebSession> session)
{
	MutexLocker ml(__mutex);

	it_advertisments_ = advertisments_.find(session);

	if (it_advertisments_ == advertisments_.end()) {
		// there is no such session. Maybe session was closed before the request is
		// processed
		return;
	}

	for (it_requests_ = advertisments_[session].begin();
	     it_requests_ != advertisments_[session].end();
	     it_requests_++) {
		if ((*it_requests_).id == advertisment_id) {
			advertisments_[session].erase(it_requests_);
			break;
		}
	}

	// sub_list_mutex_->lock();
	if (advertisments_[session].empty()) {
		session->unregister_callback(EventType::TERMINATE, shared_from_this());
		advertisments_.erase(session);
	}
}

void
Advertisment::emitt_event(EventType event_type)
{
	for (it_callables_ = callbacks_[event_type].begin();
	     it_callables_ != callbacks_[event_type].end();
	     it_callables_++) {
		(*it_callables_)->callback(event_type, shared_from_this());
	}
}

/** Callback To Be Called On Events This Instance Is Registered To
 * note that, Advertisement implements Callable interface and is notified when a
 * Session is terminated. In turn, it removes all entries of that session from
 * its registry.
 * @param event_type of event responsible for this call
 * @param event_emitter is a ptr to the instance that emitted that event
 */
void
Advertisment::callback(EventType event_type, std::shared_ptr<EventEmitter> event_emitter)
{
	// sub_list_mutex_->lock();
	MutexLocker ml(__mutex);

	try {
		// check if the event emitter was a session
		std::shared_ptr<WebSession> session;
		session = std::dynamic_pointer_cast<WebSession>(event_emitter);
		if (session != NULL) {
			if (event_type == EventType::TERMINATE) {
				// make sure the session is still there and was not deleted while
				// waiting for the mutex
				if (advertisments_.find(session) != advertisments_.end())
					advertisments_.erase(session);

				// std::cout<< "Session terminated NICELY :D" << std::endl;

				// was it the last session? if yes, Advertisment emit TERMINATTION event
				// to the Advertisment_Manager.
				if (advertisments_.empty()) {
					std::shared_ptr<Advertisment> my_self =
					  shared_from_this(); // Just to keep object alive till after its
					                      // deleted from manager
					ml.unlock();

					emitt_event(EventType::TERMINATE);

					// ml.unlock();
					// my_self->finalize();
					// std::cout<< "Advertisment topic terminated!" << std::endl;

					// finalize will need the mutex
				}
				// TODO:check if advertisment became empty and trigger the delete from
				// the owning class if that was the case
			}
		}

	} catch (Exception &e) {
		// if exception was fired it only means that the casting failed becasue the
		// emitter is not a session
	}
}
