
/***************************************************************************
 * proxy_session.h - A Session To RosBridge Server. Handles Communication To The
 *WebSession It Maps To
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

#ifndef _PROXY_SESSION_H
#define _PROXY_SESSION_H

#include "callable.h"
#include "event_emitter.h"
#include "event_type.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>

#include <memory>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

// TODO::move to commonNameSpace
typedef websocketpp::client<websocketpp::config::asio_client> Client;

namespace fawkes {
class Logger;
class Mutex;
} // namespace fawkes

class WebSession;

class ProxySession
// : public EventEmitter
// , public Callable
// , public std::enable_shared_from_this<ProxySession>
{
public:
	ProxySession();
	~ProxySession();

	// void terminate();

	// void on_open(websocketpp::connection_hdl hdl , std::shared_ptr<Client>
	// endpoint_ptr ); void on_fail(websocketpp::connection_hdl hdl); void
	// on_close(websocketpp::connection_hdl hdl);//this will be called when
	// session is closed from server
	void on_message(websocketpp::connection_hdl                                        hdl,
	                websocketpp::client<websocketpp::config::asio_client>::message_ptr msg);
	bool send(std::string const &msg);

	void set_id(int id);
	void set_connection_hdl(websocketpp::connection_hdl hdl);
	void set_endpoint(std::shared_ptr<Client> endpoint_ptr);
	void set_status(std::string status);

	int                         get_id();
	std::string                 get_status();
	websocketpp::connection_hdl get_connection_hdl();
	std::shared_ptr<WebSession> get_web_session();

	void register_web_session(std::shared_ptr<WebSession> web_session);
	void unregister_web_session();

private:
	// The websocket client session where all interactions to be mapped
	std::shared_ptr<WebSession> web_session_;
	bool                        web_session_available;

	websocketpp::connection_hdl hdl_;
	std::shared_ptr<Client>     endpoint_ptr_;

	std::string status_;
	int         session_id_;

	fawkes::Mutex *mutex_;
};

#endif