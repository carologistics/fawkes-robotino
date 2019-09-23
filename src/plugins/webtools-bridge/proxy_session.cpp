
/***************************************************************************
 * proxy_session.cpp - A Session To RosBridge Server. Handles Communication To
 *The WebSession It Maps To
 *
 *  Created: 2016
 *  Copyright  2016  MosafaGomaa
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

#include "proxy_session.h"

#include "callable.h"
#include "web_session.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

#include <exception>

using namespace fawkes;
using namespace websocketpp;

/** @class ProxySession "proxy_session.h"
 * An instance of ProxySession is created by RosBridgeProxyProcessor
 * representing a WebSocket session to the RosBridge Server. Each ProxySession
 * encapsulates the WebClientSession "WebSession" that issued a request targeted
 * for RosBridge.
 *
 * Since a ProxySession represents a single session to RosBridge. Any incoming
 * messages on this ProxySession (sent from RosBridge) will be simply forwarded
 * to the the client "WebSession" directly from the on_message() handler. This
 * simplifies the mapping, since RosBridgeProxyProcessor needs only, to care
 * about the processing of the requests coming from "WebSessions",and choose/map
 * it to a ProxySession that will handle the communication between RosBridge
 * server and the clients "WebSession".
 */

/** Constructor */
ProxySession::ProxySession() : status_("connecting")
{
	web_session_available                    = false;
	std::shared_ptr<WebSession> dumy_session = websocketpp::lib::make_shared<WebSession>();
	mutex_                                   = new fawkes::Mutex();
}

/** Destructor */
ProxySession::~ProxySession()
{
	delete mutex_;
	web_session_.reset();
}

/** On Message Handler
 * Will be called automatically when a message is received by the
 * ProxySession. Usually, this message is sent from RosBridge Server.
 * It forwards the payload of the WebSocket message (the RosBridge
 * protocol JSON) to the client's "WebSession" the ProxySession tracks.
 * @param hdl websocketpp connection handler, a pointer used to identify this
 * connection.
 * @param msg a pointer to the WebSocket message sent
 */
void
ProxySession::on_message(websocketpp::connection_hdl                                        hdl,
                         websocketpp::client<websocketpp::config::asio_client>::message_ptr msg)
{
	MutexLocker ml(mutex_);
	if (status_ != "open" && !web_session_available) {
		return;
		// throw
	}

	std::string jsonString = msg->get_payload();

	// maybe check for its validity before
	web_session_->send(jsonString);
}

/** Send A WebSocket message To RosBrirdge Server.
 * @param msg the JSON string that will be in the payload of the WebSocket
 * message.
 * @return true on succeeding
 * @todo catch exceptions and print in the log.
 */
bool
ProxySession::send(std::string const &msg)
{
	// MutexLocker ml(mutex_);
	if (status_ != "open") {
		return false;
		// throw
	}

	websocketpp::lib::error_code ec;

	// std::cout << ">TO Proxy::sending message: "<<msg << std::endl;

	endpoint_ptr_->send(hdl_, msg, websocketpp::frame::opcode::text, ec);

	if (ec) {
		// std::cout << "> Error sending message: " << ec.message() << std::endl;
		return false;
	}
	return true;
}

/** Set internal session id.
 * @param id to set
 */
void
ProxySession::set_id(int id)
{
	MutexLocker ml(mutex_);
	session_id_ = id;
}

/** Set internal websocketpp connection handler
 * @param hdl websocket connection_hdl used to identify this session
 */
void
ProxySession::set_connection_hdl(websocketpp::connection_hdl hdl)
{
	MutexLocker ml(mutex_);
	hdl_ = hdl;
}

/** Set internal websocketpp endpoint
 * @param endpoint_ptr websocket endpoint
 */
void
ProxySession::set_endpoint(std::shared_ptr<Client> endpoint_ptr)
{
	MutexLocker ml(mutex_);
	endpoint_ptr_ = endpoint_ptr;
}

/** Set internal session status
 * @param status to set
 */
void
ProxySession::set_status(std::string status)
{
	MutexLocker ml(mutex_);
	status_ = status;
}

/** Get internal id
 * @return id of the session
 */
int
ProxySession::get_id()
{
	MutexLocker ml(mutex_);
	return session_id_;
}

/** Get internal session status
 * @return status string
 */
std::string
ProxySession::get_status()
{
	MutexLocker ml(mutex_);
	return status_;
}

/** Get internal websocketpp connection handler
 * @return the websocekctpp::connection_hdl of the session
 */
websocketpp::connection_hdl
ProxySession::get_connection_hdl()
{
	MutexLocker ml(mutex_);
	return hdl_;
}

/** Get the internal websession
 * @return websession ptr of the proxied session
 */
std::shared_ptr<WebSession>
ProxySession::get_web_session()
{
	MutexLocker ml(mutex_);
	return web_session_;
}

/** set internal web session
 * @param web_session to register*/
void
ProxySession::register_web_session(std::shared_ptr<WebSession> web_session)
{
	MutexLocker ml(mutex_);
	web_session_          = web_session;
	web_session_available = true;
}

/** unregister web session*/
void
ProxySession::unregister_web_session()
{
	MutexLocker ml(mutex_);
	web_session_available                    = false;
	std::shared_ptr<WebSession> dumy_session = websocketpp::lib::make_shared<WebSession>();
	web_session_                             = dumy_session;
}

// ==>OLD CODE
// void
// ProxySession::terminate()
// {
//     std::shared_ptr <ProxySession> me= shared_from_this();
//  //   web_session_->unregister_callback(EventType::TERMINATE ,
//  shared_from_this() );

//     //emit termination event for all listeners of this session (should delete
//     all ptrs to this instance) me->emitt_event( EventType::TERMINATE );
//}

// void
// ProxySession::on_open( websocketpp::connection_hdl hdl ,
// std::shared_ptr<Client> endpoint_ptr )
// {
//     MutexLocker ml(mutex_);
//     web_session_->register_callback(EventType::TERMINATE , shared_from_this()
//     );
// 	hdl_ = hdl;
//     endpoint_ptr_=endpoint_ptr;
// 	status_="open";
// }

// void
// ProxySession::on_fail(websocketpp::connection_hdl hdl)
// {
// 	MutexLocker ml(mutex_);
// 	status_="failed";
// 	terminate();
// 	//TODO:do something about the fail
// }

// /* It Notifies anyone that registered for the session termination by directly
// calling their registed callback with the session ptr.
//  */
// void ProxySession::on_close(websocketpp::connection_hdl hdl)
// {
//  	MutexLocker ml(mutex_);

//     //std::cout << " on close " << std::endl;

//  	status_= "closed";
//  	terminate();
//}
