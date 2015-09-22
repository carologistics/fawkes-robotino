
#ifndef __PLUGINS_BRIDGE_DISPATCHER_H_
#define __PLUGINS_BRIDGE_DISPATCHER_H_


#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <thread>
#include "ros_proxy.h"

class RosProxy;

using boost::asio::ip::tcp;



class Dispatcher:  
	public boost::enable_shared_from_this<Dispatcher>
{

private:
	boost::asio::io_service					io_service_;
	boost::asio::ip::tcp::acceptor			acceptor_;
	//boost::asio::ip::tcp::socket			client_socket_;
	//boost::shared_ptr<boost::asio::ip::tcp::socket> client_socket_ptr;
	unsigned short 							client_port_;
	unsigned short 							server_port_;
	std::string 							server_host_;
	//bool									serverInit;								


	boost::asio::streambuf buff_c;
	
	



public:

	Dispatcher(unsigned short client_port,unsigned short server_port);
	~Dispatcher();
	
	void start_accept();
	void start_dispatching();
	void disconnect(const char *where, const char *reason);

	void handle_accept(const boost::system::error_code &ec);

	void handle_client_reads(const boost::system::error_code &ec);
	void write_to_client(std::string s);

	RosProxy *rosProxy_;

};

#endif


