
#ifndef __PLUGINS_BRIDGE_ROS_PROXY_H_
#define __PLUGINS_BRIDGE_ROS_PROXY_H_


#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <string>
#include <thread>

class Dispatcher;

class RosProxy:
public boost::enable_shared_from_this<RosProxy>
{

private:
//	boost::asio::io_service					io_service_;
	
	boost::asio::ip::tcp::resolver 			resolver_;
	unsigned short 							server_port_;
	std::string 							server_host_;
	//boost::shared_ptr<boost::asio::io_service>           io_service_ptr_;	
	//boost::shared_ptr<boost::asio::ip::tcp::socket> client_socket_ptr;
	boost::asio::streambuf buff_s;
	//boost::shared_ptr<Dispatcher> 			dispatcher_;

public:
	boost::asio::ip::tcp::socket 			client_socket_;
	boost::asio::ip::tcp::socket			server_socket_;

	RosProxy(boost::asio::io_service  &io_service_ , unsigned short server_port);
	~RosProxy();
	
	void start_accept();
	void start_server();
	void disconnect(const char *where, const char *reason);

	void handle_resolve(const boost::system::error_code& err,
					    boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
	void handle_connect(const boost::system::error_code &ec);
	void handle_server_reads(const boost::system::error_code &ec);
	
	void read_from_server();
	void write_to_socket(boost::asio::ip::tcp::socket &socket, std::string s);
	void write_to_server(std::string req);

	bool init_handshake(std::string msg);
	bool serverInit;

};

typedef boost::shared_ptr<RosProxy> rosProxy_ptr;

#endif


