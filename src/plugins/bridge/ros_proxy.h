
#ifndef __PLUGINS_BRIDGE_ROS_PROXY_H_
#define __PLUGINS_BRIDGE_ROS_PROXY_H_


#include <boost/asio.hpp>
#include <string>
#include <thread>


class RosProxy{

private:
	boost::asio::io_service					io_service_;
	std::thread								io_service_thread_;
	boost::asio::ip::tcp::acceptor			acceptor_;
	boost::asio::ip::tcp::socket			client_socket_;
	boost::asio::ip::tcp::socket			server_socket_;
	boost::asio::ip::tcp::resolver 			resolver_;
	unsigned short 							client_port_;
	unsigned short 							server_port_;
	std::string 							server_host_;
	bool									serverInit;								


	boost::asio::streambuf buff_c;
	boost::asio::streambuf buff_s;
	boost::asio::streambuf buff_;




public:

	RosProxy(unsigned short client_port, unsigned short server_port);
	~RosProxy();
	
	void start_accept();
	void start_server();
	void disconnect(const char *where, const char *reason);

	void handle_accept(const boost::system::error_code &ec);
	void handle_resolve(const boost::system::error_code& err,
					    boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
	void handle_connect(const boost::system::error_code &ec);
	void handle_read(const boost::system::error_code &ec);
	void handle_server_reads(const boost::system::error_code &ec);
	void handle_client_reads(const boost::system::error_code &ec);
	



	std::string read_connection_request();
	void read_from_socket(boost::asio::ip::tcp::socket &socket);
	void read_from_client_to_server();
	void read_from_server_to_client();
	void write_to_socket(boost::asio::ip::tcp::socket &socket);
	void write_to_server();
	void write_to_client();

};

#endif


