/***************************************************************************
 *  websocket_server.h - Web Socket Server
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

#ifndef _WEBSOCKET_SERVER_H
#define _WEBSOCKET_SERVER_H

#include <map>
#include <memory>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace fawkes {
class Logger;
class Mutex;
} // namespace fawkes

class WebSession;
class BridgeManager;

#ifndef _SERVER
#	define _SERVER
typedef websocketpp::server<websocketpp::config::asio> server;
#endif

class WebSocketServer
{
public:
	WebSocketServer(fawkes::Logger *logger, std::shared_ptr<BridgeManager> bridge_manager);
	~WebSocketServer();

	void finalize();

	bool on_validate(websocketpp::connection_hdl hdl);
	void on_open(websocketpp::connection_hdl hdl);
	void on_close(websocketpp::connection_hdl hdl);
	void on_message(websocketpp::connection_hdl                                 hdl,
	                websocketpp::server<websocketpp::config::asio>::message_ptr web_msg);

	void run(uint16_t port);

private:
	std::map<websocketpp::connection_hdl,
	         std::shared_ptr<WebSession>,
	         std::owner_less<websocketpp::connection_hdl>>
	  hdl_ids_;

	std::shared_ptr<websocketpp::lib::thread> m_thread;
	std::shared_ptr<server>                   m_server;
	std::shared_ptr<WebSession> tmp_session_; // this only serve to collect the session data before
	                                          // intializing the dispaticher
	std::shared_ptr<BridgeManager> bridge_manager_;

	fawkes::Logger *logger_;
	fawkes::Mutex * mutex_;

	unsigned int m_next_sessionid;
	bool         finalized_;
	bool         running_;
};

#endif