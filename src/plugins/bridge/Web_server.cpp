#include <iostream>
#include <map>
#include <exception>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;



struct connection_data {
    int sessionid;
    std::string name;
    std::map<std::string,std::string> http_req;
};

class Web_server {
public:
    Web_server() : m_next_sessionid(1) {
        m_server.init_asio();

        m_server.set_open_handler(bind(&Web_server::on_open,this,::_1));
        m_server.set_close_handler(bind(&Web_server::on_close,this,::_1));
        m_server.set_message_handler(bind(&Web_server::on_message,this,::_1,::_2));
        m_server.set_validate_handler(bind(&Web_server::on_validate,this,::_1));
    }

    bool on_validate(connection_hdl hdl){

    	server::connection_ptr con=m_server.get_con_from_hdl(hdl);
    	
    	connection_data data;

    	data.http_req["host"]=con->get_host();
    	data.http_req["port"]=con->get_port();
    	data.http_req["origin"]=con->get_origin();

    	std::vector<std::string> http_headers;
    	http_headers.push_back("Accept-Encoding");
    	http_headers.push_back("Accept-language");
    	http_headers.push_back("Cache-Control");
    	http_headers.push_back("Connection");
    	http_headers.push_back("Sec-WebSocket-Extensions");
    	http_headers.push_back("Sec-WebSocket-Key");
    	http_headers.push_back("Sec-WebSocket-Version");
    	http_headers.push_back("User-Agent");
    
		for (std::vector<std::string>::const_iterator i = http_headers.begin(); i != http_headers.end(); ++i)
    	data.http_req[*i]=con->get_request_header(*i);

    	//I think this just copies the data ...so be carful later when extending the connection_metada object
        m_connections[hdl] = data;

		return true;

    }

    void on_open(connection_hdl hdl) {
        connection_data& data = get_data_from_hdl(hdl);

        data.sessionid = m_next_sessionid++;
        data.name = "";

        for (std::map<std::string,std::string>::const_iterator i = data.http_req.begin(); i != data.http_req.end(); ++i)
    	std::cout<< i->first << "::::::" << i->second << std::endl;

        }

    void on_close(connection_hdl hdl) {
        connection_data& data = get_data_from_hdl(hdl);

        std::cout << "Closing connection " << data.name
                  << " with sessionid " << data.sessionid << std::endl;

        m_connections.erase(hdl);
    }

    void on_message(connection_hdl hdl, server::message_ptr msg) {
        connection_data& data = get_data_from_hdl(hdl);

        if (data.name == "") {
            data.name = msg->get_payload();
            std::cout << "Setting name of connection with sessionid "
                      << data.sessionid << " to " << data.name << std::endl;
        } else {
            std::cout << "Got a message from connection " << data.name
                      << " with sessionid " << data.sessionid << std::endl;
        }
    }

    connection_data& get_data_from_hdl(connection_hdl hdl) {
        auto it = m_connections.find(hdl);

        if (it == m_connections.end()) {
            // this connection is not in the list. This really shouldn't happen
            // and probably means something else is wrong.
            throw std::invalid_argument("No data available for session");
        }

        return it->second;
    }

    void run(uint16_t port) {
        m_server.listen(port);
        m_server.start_accept();
        m_server.run();
    }
private:
    typedef std::map<connection_hdl,connection_data,std::owner_less<connection_hdl>> con_list;

    int m_next_sessionid;
    server m_server;
    con_list m_connections;
};

int main() {
    Web_server server;
    server.run(8080);
}
