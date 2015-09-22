
#include "ros_proxy.h"
#include <boost/bind.hpp>
#include <exception> 
#include <boost/lexical_cast.hpp>

using namespace boost::asio;



RosProxy::RosProxy(unsigned short server_port):
		server_socket_(io_service_)
		,resolver_(io_service_)
		,server_port_(server_port)
		,serverInit(false)
		{
		start_server();
		this->io_service_.run();
}



RosProxy::~RosProxy()
{
	boost::system::error_code err;
  	server_socket_.shutdown(ip::tcp::socket::shutdown_both, err);
  	server_socket_.close();
  	io_service_.stop();
  	io_service_thread_.join();
  	

}
void RosProxy::disconnect(const char *where, const char *reason){

	std::cout << "Disconnected From ";
	std::cout << *where;
	std::cout << "Coz ";
	std::cout << *reason;

  boost::system::error_code ec;
   server_socket_.shutdown(ip::tcp::socket::shutdown_both, ec);
  server_socket_.close();

}



void RosProxy::start_server(){

  ip::tcp::resolver::query query("localhost", boost::lexical_cast<std::string>(server_port_));
  resolver_.async_resolve(query,
			  boost::bind(&RosProxy::handle_resolve, this,
				      boost::asio::placeholders::error,
				      boost::asio::placeholders::iterator));
}
void
RosProxy::handle_resolve(const boost::system::error_code& err,
					    ip::tcp::resolver::iterator endpoint_iterator)
{
  if (! err) {
    // Attempt a connection to each endpoint in the list until we
    // successfully establish a connection.
#if BOOST_ASIO_VERSION > 100409
    boost::asio::async_connect(server_socket_, endpoint_iterator,
#else
    server_socket_.async_connect(*endpoint_iterator,
#endif
			       boost::bind(&RosProxy::handle_connect, this,
					   boost::asio::placeholders::error));
  } else {
    disconnect("handle_resolve", err.message().c_str());
  }
}
void
RosProxy::handle_connect(const boost::system::error_code &err)
{
  if (! err) {
//	wiat for hand shake
  	read_from_server_to_client()
  }
  else {
    disconnect("handle_connect", err.message().c_str());
  }
}


void
RosProxy::handle_server_reads(const boost::system::error_code &ec){


	if(!ec){
		//check if the socket is alive
		write_to_socket(client_socket_);
		read_from_server();
	}
	else {
    disconnect("handle_server_reads", ec.message().c_str());
  }

}
bool RosProxy::init_handshake(boost::asio::ip::tcp::socket &client_socket){

	client_socket_=client_socket;
	boost::system::error_code ec;

	//intiat the handshake with the rosBridge
		boost::asio::read(client_socket_, buff_c, boost::asio::transfer_at_least(1),ec);
//todo:check the msg to confirm
	 	std::string s="";  
		std::ostringstream ss;
		ss << &buff_c;
		s = ss.str();

	 	write_to_server(s);

//a temp solution for the no wait....

//	 	boost::asio::read(client_server_, buff_s, boost::asio::transfer_at_least(1),
//	 	boost::bind(&RosProxy::handle_handshak_req, this,
//	 				   boost::asio::placeholders::error);
//
//	 	write_to_socket(client_socket_);
//
//	 	read_from_server_to_client();

//check if success and return accourdingly
	 	return true;

}
void
RosProxy::read_from_server(){

  std::cout << "Waiting to read from  server! \n";
		 boost::asio::async_read(server_socket_, buff_s, boost::asio::transfer_at_least(1),
		 	boost::bind(&RosProxy::handle_server_reads, this,
		 				   boost::asio::placeholders::error)
		 	);

}



void RosProxy::write_to_server(std::string s){
	boost::system::error_code ec;

	size_t t =boost::asio::buffer_size(boost::asio::buffer(s));
	
	std::cout << "Writing to server!";
	std::cout << t;
	std::cout << "bytes \n!";


	boost::asio::write(server_socket_,boost::asio::buffer(s) , ec);
  if(ec){
  		std::cout << "Failed to write! \n";
  }

}
void
RosProxy::write_to_socket(boost::asio::ip::tcp::socket &socket)
{
	boost::system::error_code ec;
	std::string s="";  
	std::ostringstream ss;
	ss << &buff_s;
	s = ss.str();
	size_t t =boost::asio::buffer_size(boost::asio::buffer(s));
	

	std::cout << "Writing to client!";
	std::cout << t;
	std::cout << "bytes \n!";

 	boost::asio::write(socket,boost::asio::buffer(s) , ec);
	if(ec){
	  		std::cout << "Failed to write! \n";
	  }
  
}

