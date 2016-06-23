/*
 * Copyright (c) 2014, Peter Thorson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the WebSocket++ Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL PETER THORSON BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// **NOTE:** This file is a snapshot of the WebSocket++ utility client tutorial.
// Additional related material can be found in the tutorials/utility_client
// directory of the WebSocket++ repository.

#include <logging/logger.h>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>

#include "interfaces/idispatcher.h"
#include "interfaces/ibridge.h"



 using namespace fawkes;


typedef websocketpp::client<websocketpp::config::asio_client> client;
class connection_metadata {
public:
    typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

    connection_metadata(int id, websocketpp::connection_hdl hdl, std::string uri,  websocketpp::lib::shared_ptr<Idispatcher> dispatcher)
      : m_id(id)
      , m_hdl(hdl)
      , m_status("Connecting")
      , m_uri(uri)
      , m_server("N/A")
      , m_dispatcher(dispatcher)

    {}

    void on_open(client * c, websocketpp::connection_hdl hdl) {
        m_status = "Open";

        client::connection_ptr con = c->get_con_from_hdl(hdl);
        m_server = con->get_response_header("Server");
    }

    void on_fail(client * c, websocketpp::connection_hdl hdl) {
        m_status = "Failed";

        client::connection_ptr con = c->get_con_from_hdl(hdl);
        m_server = con->get_response_header("Server");
        m_error_reason = con->get_ec().message();
    }
    
    void on_close(client * c, websocketpp::connection_hdl hdl) {
        m_status = "Closed";
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        std::stringstream s;
        s << "close code: " << con->get_remote_close_code() << " (" 
          << websocketpp::close::status::get_string(con->get_remote_close_code()) 
          << "), close reason: " << con->get_remote_close_reason();
        m_error_reason = s.str();
    }

    void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
        if (msg->get_opcode() == websocketpp::frame::opcode::text) {
              //std::cout<< "forwarding to web.." << std::endl;
              //std::cout<< msg->get_payload()<<std::endl;
              m_dispatcher->send_to_web(msg->get_payload());
        } else {
            //std::cout<< "forwarding to web HEX" << std::endl;
           // m_messages.push_back("<< " + websocketpp::utility::to_hex(msg->get_payload()));
        }
    }

    websocketpp::connection_hdl get_hdl() const {
        return m_hdl;
    }
    
    int get_id() const {
        return m_id;
    }
    
    std::string get_status() const {
        return m_status;
    }

private:
    int m_id;
    websocketpp::connection_hdl m_hdl;
    std::string m_status;
    std::string m_uri;
    std::string m_server;
    std::string m_error_reason;
    websocketpp::lib::shared_ptr<Idispatcher> m_dispatcher;
};



//---------------------------------------------------ROSENDPOINT

using namespace fawkes;

class ros_proxy : public Ibridge
{


public:
    typedef websocketpp::lib::shared_ptr<ros_proxy> ptr;

    ros_proxy (fawkes::Logger  *logger, websocketpp::lib::shared_ptr<Idispatcher> dispatcher) 
    : m_next_id(0)
    , m_dispatcher(dispatcher)
    , logger_(logger)
     {

        type= bridgeType::ROS_BRIDGE;
        m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        m_endpoint.clear_error_channels(websocketpp::log::elevel::all);

        m_endpoint.init_asio();
        m_endpoint.start_perpetual();
    }

    ~ros_proxy() {
        m_endpoint.stop_perpetual();

        //std::cout << "> Closing connection " << metadata_ptr->get_id() << std::endl;
        
        websocketpp::lib::error_code ec;
        m_endpoint.close(metadata_ptr->get_hdl(), websocketpp::close::status::going_away, "", ec);
        if (ec) {
            //std::cout << "> Error closing connection " << metadata_ptr->get_id() << ": "  
                      << ec.message() << std::endl;
        }
        m_thread->join();
    }

    int connect(std::string const & uri) {
        websocketpp::lib::error_code ec;

        client::connection_ptr con = m_endpoint.get_connection(uri, ec);


        if (ec) {
            logger_->log_info("Webtools-bridge:"," Connect initialization error: %s" , ec.message().c_str());
            return -1;
        }

        int new_id = m_next_id++;
        metadata_ptr = websocketpp::lib::make_shared<connection_metadata>(new_id, con->get_handle(), uri,m_dispatcher);

        con->set_open_handler(websocketpp::lib::bind(
            &connection_metadata::on_open,
            metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
        ));
        con->set_fail_handler(websocketpp::lib::bind(
            &connection_metadata::on_fail,
            metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
        ));
        con->set_close_handler(websocketpp::lib::bind(
            &connection_metadata::on_close,
            metadata_ptr,
            &m_endpoint,
            websocketpp::lib::placeholders::_1
        ));
        con->set_message_handler(websocketpp::lib::bind(
            &connection_metadata::on_message,
            metadata_ptr,
            websocketpp::lib::placeholders::_1,
            websocketpp::lib::placeholders::_2
        ));

        m_endpoint.connect(con);

        return new_id;
    }

    void close(int id, websocketpp::close::status::value code, std::string reason) {
        websocketpp::lib::error_code ec;
        m_endpoint.close(metadata_ptr->get_hdl(), code, reason, ec);
        if (ec) {
            //std::cout << "> Error initiating close: " << ec.message() << std::endl;
        }
    }


    void send(std::string message) {
        websocketpp::lib::error_code ec;
        m_endpoint.send(metadata_ptr->get_hdl(), message, websocketpp::frame::opcode::text, ec);
        if (ec) {
            //std::cout << "> Error sending message to ros: " << ec.message() << std::endl;
            return;
        }

    }

    //---iBridge implementation

    bool init(){
        m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &m_endpoint);        
        int id = connect("ws://localhost:9090");
         if (id != -1) {
            logger_->log_info("Webtools-bridge:" , "> Created connection with id " );
            return true;
        } 
        return false;
    }

    void process_request(std::string msg){
        send(msg);
    }



private:
    connection_metadata::ptr metadata_ptr;

    client m_endpoint;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
    int m_next_id;

    websocketpp::lib::shared_ptr<Idispatcher>       m_dispatcher;

    fawkes::Logger                                   *logger_;

};
