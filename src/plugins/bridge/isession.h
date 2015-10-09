#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;

class Isession{

public:

	virtual	int 				get_id()=0;
	virtual	std::string 			get_name()=0;
	virtual std::string 			get_status()=0; 

	// maybe replace this later to achive more abstraction with the operations needed and not pointer interaction
	virtual	server::connection_ptr get_connection_ptr()=0;

	virtual void send(std::string msg)=0;
};