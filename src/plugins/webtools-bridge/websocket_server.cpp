/***************************************************************************
 *  websocekt_server.cpp - Web Socket Server
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

#include "websocket_server.h"

#include "bridge_manager.h"
#include "web_session.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>

#include <exception>

using websocketpp::connection_hdl;
using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

using namespace fawkes;

/** @class WebSocketServer
 * Websocket server for webtools-bridge. Implementation based on websocketpp
 * lib. Implements the needed event handlers for sessions and wraps websocketpp
 * session into a more meaningful internal representation (ie, WebSession). The
 * server contains a 'bridge manager' instance where JSON messages are passed,
 * along with their initiating session, for further processing.
 *
 * @author Mostafa Gomaa
 */

/** Constructor
 * @param logger Fawkes logger
 * @param bridge_manager entry point to dispatching requests
 */
WebSocketServer::WebSocketServer(fawkes::Logger *               logger,
                                 std::shared_ptr<BridgeManager> bridge_manager)
: m_next_sessionid(1), finalized_(false)
{
	logger_         = logger;
	bridge_manager_ = bridge_manager;
	m_server        = websocketpp::lib::make_shared<server>();

	m_server->init_asio();
	m_server->set_open_handler(bind(&WebSocketServer::on_open, this, ::_1));
	m_server->set_close_handler(bind(&WebSocketServer::on_close, this, ::_1));
	m_server->set_validate_handler(bind(&WebSocketServer::on_validate, this, ::_1));
	m_server->clear_access_channels(websocketpp::log::alevel::all);
	m_server->set_reuse_addr(true);

	mutex_ = new fawkes::Mutex();
}

/** Destructor */
WebSocketServer::~WebSocketServer()
{
	if (!finalized_) {
		finalize();
	}

	// TODO: What if the asio::run thread gets stuck while holding the mutex
	//(in on_message of ex while processing  a request), am not sure if it could
	// stop and join
	if (!m_server->stopped()) {
		m_server->stop();
		usleep(100000);
		logger_->log_info("WebServer", "Stopping Asio");
	}

	if (m_thread != NULL)
		m_thread->join();

	delete mutex_;
	m_server.reset();
}

/** Start listening for connection of some port
 * @param port to listen to*/
void
WebSocketServer::run(uint16_t port)
{
	m_server->listen(port);
	m_server->start_accept();
	m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&server::run, m_server);
}

/** Finalize the server instance */
void
WebSocketServer::finalize()
{
	MutexLocker ml(mutex_);
	if (finalized_) {
		return;
	}

	logger_->log_info("WebServer:", " Finalizing");

	websocketpp::lib::error_code ec;

	for (std::map<connection_hdl, std::shared_ptr<WebSession>>::iterator it = hdl_ids_.begin();
	     it != hdl_ids_.end();) {
		m_server->close(it->first, websocketpp::close::status::going_away, "", ec);
		it++;
		// usleep(5000); // wait for session closing to terminate cleanl by handler.
		// Or just count on the termination in the destrouctor
		if (ec) {
			logger_->log_info("WebServer:", " Error closing connection : : %s", ec.message().c_str());
		}
	}

	while (!hdl_ids_.empty()) {
		logger_->log_info("WebServer", "emptying hdls");
		hdl_ids_.begin()->second->on_terminate(); // Should garanty the all sessions
		                                          // instanes went out of scope
		hdl_ids_.erase(hdl_ids_.begin());
	}

	finalized_ = true;
}

/** Called on when a connection is validated.
 * @param hdl websocket connection_hdl used to identify this session
 * @return true on succeeding
 */
bool
WebSocketServer::on_validate(connection_hdl hdl)
{
	MutexLocker ml(mutex_);
	if (finalized_) {
		return false;
	}

	tmp_session_ = websocketpp::lib::make_shared<WebSession>();
	tmp_session_->set_status("validating"); // todo::use status codes from websocketpp

	server::connection_ptr con = m_server->get_con_from_hdl(hdl);
	return true;
}

/** Called on establishment of a connection.
 * @param hdl websocket connection_hdl used to identify this session
 */
void
WebSocketServer::on_open(connection_hdl hdl)
{
	MutexLocker ml(mutex_);
	if (finalized_) {
		return;
	}

	tmp_session_->set_connection_hdl(hdl);
	tmp_session_->set_endpoint(m_server);
	tmp_session_->set_id(m_next_sessionid);
	tmp_session_->set_status("open"); // todo::use status codes from websocketpp

	hdl_ids_[hdl] = tmp_session_;

	m_server->get_con_from_hdl(hdl)->set_message_handler(
	  bind(&WebSocketServer::on_message, this, ::_1, ::_2));

	m_next_sessionid++;
	// ForDebuging:: Print on http req
	// for (std::map<std::string,std::string>::const_iterator i =
	// tmp_session_->http_req.begin(); i != tmp_session_->http_req.end(); ++i)
	// //std::cout<< i->first << "::::::" << i->second << std::endl;

	logger_->log_info("Webtools-Bridge:", "on open");
}

/** Called on closing of a connection.
 * @param hdl websocket connection_hdl used to identify this session
 */
void
WebSocketServer::on_close(connection_hdl hdl)
{
	MutexLocker ml(mutex_);
	if (finalized_) {
		return;
	}

	tmp_session_->set_status("close");

	websocketpp::lib::error_code ec;

	auto it = hdl_ids_.find(hdl);

	// TODO:: just do the termination if its there. do not throw and nothing
	// otherwise
	if (it == hdl_ids_.end()) {
		// this connection is not in the list. This really shouldn't happen
		// and probably means something else is wrong.
		throw std::invalid_argument("No data available for session");
	}

	int session_id = it->second->get_id();
	// TODO::replace by smarter registration mechanism
	it->second->on_terminate();

	std::cout << "Closing connection  with sessionid " << session_id << std::endl;

	hdl_ids_.erase(hdl);
}

/** Finds the requesting sessions by its hdl.
 * Extracts the pay load from the msg.
 * Forwards both to incoming of the bridge manger.
 * @param hdl websocket connection_hdl used to identify this session
 * @param web_msg ptr to the message
 */
void
WebSocketServer::on_message(connection_hdl                                              hdl,
                            websocketpp::server<websocketpp::config::asio>::message_ptr web_msg)
{
	MutexLocker ml(mutex_);
	if (finalized_) {
		return;
	}

	std::shared_ptr<WebSession> session = hdl_ids_[hdl];

	std::string jsonString = web_msg->get_payload();

	logger_->log_info("Webtools-Bridge", "Msg Received!");

	try {
		bridge_manager_->incoming(jsonString, session);
	} catch (fawkes::Exception &e) {
		logger_->log_error("Webtools-Bridge", e);
	}
}
