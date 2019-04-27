/***************************************************************************
 *  web_session.h -  Session object to keep track of a unique session.
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

#ifndef _WEB_SESSION_H
#define _WEB_SESSION_H

#include "event_emitter.h"
#include "event_type.h"

#include <list>
#include <map>
#include <memory>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace fawkes {
class Mutex;
}

class WebSession;

#ifndef _SERVER
#	define _SERVER
typedef websocketpp::server<websocketpp::config::asio> server;
#endif

class WebSession : public EventEmitter, public std::enable_shared_from_this<WebSession>
{
public:
	WebSession();
	~WebSession();

	void set_connection_hdl(websocketpp::connection_hdl hdl);
	void set_endpoint(std::shared_ptr<server> endpoint_ptr);
	void set_id(int id);
	void set_status(std::string status);

	server::connection_ptr get_connection_ptr();
	int                    get_id();
	std::string            get_status();

	bool send(std::string msg);

	void on_terminate(); // this will be called when session is closed from server

	void emitt_event(EventType event_type);

private:
	websocketpp::connection_hdl hdl_;
	std::shared_ptr<server>     endpoint_ptr_;

	std::string session_name_;
	std::string status_;
	int         session_id_;

	fawkes::Mutex *mutex_;
};

#endif