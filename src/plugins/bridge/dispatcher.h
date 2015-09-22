
#ifndef __PLUGINS_BRIDGE_DISPATCHER_H_
#define __PLUGINS_BRIDGE_DISPATCHER_H_


#include <boost/asio.hpp>
#include <string>
#include <thread>

class ros_proxy;

class dispatcher{

private:
	boost::asio::io_service					io_service_;
	std::thread								io_service_thread_;
	boost::asio::ip::tcp::acceptor			acceptor_;
	boost::asio::ip::tcp::socket			client_socket_;
	unsigned short 							client_port_;
	unsigned short 							server_port_;
	std::string 							server_host_;
	bool									serverInit;								


	boost::asio::streambuf buff_c;
	boost::asio::streambuf buff_s;
	boost::asio::streambuf buff_;

	ros_proxy *ros_proxy_;



public:

	dispatcher(unsigned short client_port);
	~dispatcher();
	
	void start_accept();
	void start_dispatching();
	void disconnect(const char *where, const char *reason);

	void handle_accept(const boost::system::error_code &ec);

	void handle_client_reads(const boost::system::error_code &ec);


};

#endif


