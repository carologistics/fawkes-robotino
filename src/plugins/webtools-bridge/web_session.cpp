/***************************************************************************
 *  web_session.cpp -  Session object to keep track of a unique session.
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

#include "web_session.h"

#include "callable.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

#include <exception>

using namespace fawkes;

/** @class WebSession
 * Intermediate more meaningful representation of a WebSockt Session.
 * Contains some of the members and handlers needed by websocketpp along
 * with some domain specific requirements.
 *
 * @author Mostafa Gomaa
 */
/** Constructor*/
WebSession::WebSession() : EventEmitter(), status_("N/A")
{
}

/** Destructor */
WebSession::~WebSession()
{
}

/** Set internal websocketpp connection handler
 * @param hdl websocket connection_hdl used to identify this session
 */
void
WebSession::set_connection_hdl(websocketpp::connection_hdl hdl)
{
	hdl_ = hdl;
}

/** Set internal websocketpp endpoint
 * @param endpoint_ptr websocket endpoint
 */
void
WebSession::set_endpoint(std::shared_ptr<server> endpoint_ptr)
{
	endpoint_ptr_ = endpoint_ptr;
}

/** Set internal session id
 * @param id to set
 */
void
WebSession::set_id(int id)
{
	session_id_ = id;
}

/** Set internal session status
 * @param status to set
 */
void
WebSession::set_status(std::string status)
{
	status_ = status;
}

/** Get internal id
 * @return id of the session
 */
int
WebSession::get_id()
{
	return session_id_;
}

/** Get internal session status
 * @return status string
 */
std::string
WebSession::get_status()
{
	return status_;
}

/** Get internal websocketpp connection handler
 * @return the websocekctpp::connection_hdl of the session
 */
server::connection_ptr
WebSession::get_connection_ptr()
{
	return endpoint_ptr_->get_con_from_hdl(hdl_);
}

/**Send WebSocket message to a session
 * @param msg the JSON string that will be in the payload of the WebSocket
 * message.
 * @return true on success
 */
bool
WebSession::send(std::string msg)
{
	// MutexLocker ml(mutex_);

	websocketpp::lib::error_code ec;

	// std::cout << ">TO WEB::sending message: " << std::endl;
	endpoint_ptr_->send(hdl_, msg, websocketpp::frame::opcode::text, ec);

	if (ec) {
		// std::cout << "> Error sending message: " << ec.message() << std::endl;
		return false;
	}
	return true;
}

/** This session has been closed from the server
 * This method is called whenever the session is closed by the server.
 * It Notifies anyone that registered for the session termination by directly
 * calling their resisted callback with the session ptr.
 */
void
WebSession::on_terminate()
{
	std::shared_ptr<WebSession> me = shared_from_this();

	me->emitt_event(EventType::TERMINATE);
}

void
WebSession::emitt_event(EventType event_type)
{
	for (it_callables_ = callbacks_[event_type].begin();
	     it_callables_ != callbacks_[event_type].end();) {
		(*it_callables_)->callback(event_type, shared_from_this());

		// this is dangerous ..i am assuming the the callable obj is still there
		// after the callback
		if (event_type == EventType::TERMINATE) {
			it_callables_ = callbacks_[event_type].erase(it_callables_);
		} else {
			it_callables_++;
		}
	}
}
