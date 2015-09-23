
#ifndef __PLUGINS_BRIDGE_ROS_PROXY_H_
#define __PLUGINS_BRIDGE_ROS_PROXY_H_


#include <boost/asio.hpp>
#include <string>
#include <thread>

class RosProxy
{

private:
//	boost::asio::io_service					io_service_;
	
	boost::asio::ip::tcp::resolver 			resolver_;
	unsigned short 							server_port_;
	std::string 							server_host_;

	boost::asio::streambuf buff_s;
	
public:
	boost::asio::ip::tcp::socket 			client_socket_;
	boost::asio::ip::tcp::socket			server_socket_;

	bool 									serverInit;

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
	void process_req(std::string req);
	
	bool check_rosBridge_alive();

};

//typedef boost::shared_ptr<RosProxy> rosProxy_ptr;

#endif


