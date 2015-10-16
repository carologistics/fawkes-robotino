#include <iostream>
#include <map>
#include <exception>
#include <websocketpp/common/thread.hpp>
#include <dispatcher.h>
#include <web_session.h>

using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;


class Web_server
{
public:
    Web_server() : m_next_sessionid(1) {
        m_server=websocketpp::lib::make_shared<server>();

        m_server->init_asio();
        m_server->set_open_handler(bind(&Web_server::on_open,this,::_1));
        m_server->set_close_handler(bind(&Web_server::on_close,this,::_1));
        m_server->set_validate_handler(bind(&Web_server::on_validate,this,::_1));        
      }

    ~Web_server() {
        m_thread->join();
 }


    bool on_validate(connection_hdl hdl){
        tmp_session_= websocketpp::lib::make_shared<web_session>();
        tmp_session_->set_status("validating");//todo::use status codes from websocketpp

        server::connection_ptr con=m_server->get_con_from_hdl(hdl);

        std::vector<std::string> register_headers;
        register_headers.push_back("Accept-Encoding");
        register_headers.push_back("Accept-language");
        register_headers.push_back("Cache-Control");
        register_headers.push_back("Connection");
        register_headers.push_back("Sec-WebSocket-Extensions");
        register_headers.push_back("Sec-WebSocket-Key");
        register_headers.push_back("Sec-WebSocket-Version");
        register_headers.push_back("User-Agent");

        tmp_session_->http_req["host"]=con->get_host();
        tmp_session_->http_req["port"]=con->get_port();
        tmp_session_->http_req["origin"]=con->get_origin();

        for (std::vector<std::string>::const_iterator i = register_headers.begin(); i != register_headers.end(); ++i)
        tmp_session_->http_req[*i]=con->get_request_header(*i);

        return true;
    }

    void on_open(connection_hdl hdl) {

        hdl_ids_[hdl]=m_next_sessionid;

        tmp_session_->set_connection_hdl(hdl);
        tmp_session_->set_endpoint(m_server);
        tmp_session_->set_id(m_next_sessionid);
        tmp_session_->set_name("web_session_tmp_name");

        dispatchers_[m_next_sessionid]= websocketpp::lib::make_shared<Dispatcher>(tmp_session_);
        dispatchers_[m_next_sessionid]->start();
        m_next_sessionid++;
        //ForDebuging:: Print on http req 
        // for (std::map<std::string,std::string>::const_iterator i = tmp_session_->http_req.begin(); i != tmp_session_->http_req.end(); ++i)
        // std::cout<< i->first << "::::::" << i->second << std::endl;
        }

    void on_close(connection_hdl hdl) {

        //TODO:: Find the id of this hdl and distory dispatcher instance with all its internals
       auto it = hdl_ids_.find(hdl);

       if(it == hdl_ids_.end()){
        // this connection is not in the list. This really shouldn't happen
            // and probably means something else is wrong.
            throw std::invalid_argument("No data available for session");
       }

       int session_id=it->second;

       std::cout << "Closing connection  with sessionid " << session_id << std::endl;

       hdl_ids_.erase(hdl);
       dispatchers_.erase(session_id);
    }


    void run(uint16_t port) {
        m_server->listen(port);
        m_server->start_accept();
        m_thread = websocketpp::lib::make_shared<websocketpp::lib::thread>(&server::run, m_server);
    }

private:
    typedef std::map<connection_hdl,int,std::owner_less<connection_hdl>>    hdl_list;
    typedef std::map<int, websocketpp::lib::shared_ptr<Dispatcher> >        disp_list;
    
    hdl_list                                                                hdl_ids_;
    disp_list                                                               dispatchers_;

    websocketpp::lib::shared_ptr<server>                                    m_server;
    websocketpp::lib::shared_ptr<web_session>                               tmp_session_; //this only serve to collect the session data before intializing the dispaticher
    
    int                                                                     m_next_sessionid;

    websocketpp::lib::shared_ptr<websocketpp::lib::thread>                  m_thread;
};



int main() {
    websocketpp::lib::shared_ptr<Web_server> web_server=websocketpp::lib::make_shared<Web_server>();
    web_server->run(6060);
}



// int main() {

//     websocketpp::lib::shared_ptr<Web_server> web_server=websocketpp::lib::make_shared<Web_server>();
//     ros_endpoint::ptr rosbridge_ptr= websocketpp::lib::make_shared<ros_endpoint>();
//     websocketpp::lib::shared_ptr<Dispatcher> dispatcher=websocketpp::lib::make_shared<Dispatcher>();

//     dispatcher->register_ros_endpoint(rosbridge_ptr);
//     web_server->register_dispatcher(dispatcher);
    
//     web_server->run(6060);

//     rosbridge_ptr->run();
//     int id =  rosbridge_ptr->connect("ws://localhost:9090");
//     if (id != -1) {
//         std::cout << "> Created connection with id " << id << std::endl;
//    }

// }



    // void register_dispatcher(websocketpp::lib::shared_ptr<Dispatcher> dispatcher){
    //     dispatcher_=dispatcher;

    //     websocketpp::lib::shared_ptr<Iendpoint> wrappedEndpoint= std::dynamic_pointer_cast<Iendpoint>(shared_from_this());
    //     dispatcher->register_web_endpoint(wrappedEndpoint);  
    // }


 //    void on_message(connection_hdl hdl, server::message_ptr msg) {
 //         std::cout << "Got a message from connection "  << std::endl;

    // dispatcher_->dispatch_msg(msg->get_payload());
 //    }


 //    connection_data& get_data_from_hdl(connection_hdl hdl) {
 //        auto it = m_connections.find(hdl);

 //        if (it == m_connections.end()) {
 //            // this connection is not in the list. This really shouldn't happen
 //            // and probably means something else is wrong.
 //            throw std::invalid_argument("No data available for session");
 //        }

 //        return it->second;
 //    }

 //        void send(std::string message) {
 //        websocketpp::lib::error_code ec;
        
 //        m_server.send(m_connections.begin()->first, message, websocketpp::frame::opcode::text, ec);
 //        if (ec) {
 //            std::cout << "> Error sending message: " << ec.message() << std::endl;
 //            return;
 //        }
        
 //    }
