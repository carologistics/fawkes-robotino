

//#include "dispatcher.h"
#include "ros_proxy.h"
#include <boost/bind.hpp>
#include <exception> 
#include <boost/lexical_cast.hpp>


#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>


using namespace boost::asio;



RosProxy::RosProxy(boost::asio::io_service& io_service_,unsigned short server_port):
		server_socket_(io_service_)
		,client_socket_(io_service_)
		,resolver_(io_service_)
		,server_port_(server_port)
		,serverInit(false)
		{
		start_server();
			std::cout <<"init1 \n";
}



RosProxy::~RosProxy()
{
	boost::system::error_code err;
  	server_socket_.shutdown(ip::tcp::socket::shutdown_both, err);
  	server_socket_.close();
  	client_socket_.shutdown(ip::tcp::socket::shutdown_both, err);
  	client_socket_.close();
  
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
std::cout <<"starting on";
std::cout << server_port_;
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
 std::cout << "resolving \n";
    // Attempt a connection to each endpoint in the list until we
    // successfully establish a connection.
#if BOOST_ASIO_VERSION > 100409
    boost::asio::async_connect(server_socket_, endpoint_iterator,
#else
    server_socket_.async_connect(*endpoint_iterator,
#endif
			       boost::bind(&RosProxy::handle_connect, this,
					   boost::asio::placeholders::error));




//     boost::system::error_code ec = boost::asio::error::would_block;
//   server_socket_.async_connect(*endpoint_iterator,
//   			       boost::lambda::var(ec) = boost::lambda::_1);

//   // Block until the asynchronous operation has completed.
//   do {
//     io_service_->run_one();
// #if BOOST_VERSION >= 105400 && BOOST_VERSION < 105500
//     // Boost 1.54 has a bug that causes async_connect to report success
//     // if it cannot connect at all to the other side, cf.
//     // https://svn.boost.org/trac/boost/ticket/8795
//     // Work around by explicitly checking for connected status
//     if (! ec) {
//       server_socket_.remote_endpoint(ec);
//       if (ec == boost::system::errc::not_connected) {
//         // continue waiting for timeout
//         ec = boost::asio::error::would_block;
// 	server_socket_.async_connect(*endpoint_iterator,
//                                      boost::lambda::var(ec) = boost::lambda::_1);
//       }
//     }
// #endif
//   } while (ec == boost::asio::error::would_block);

//   // Determine whether a connection was successfully established.
// 	  if (ec || ! server_socket_.is_open()) {
// 		    disconnect("handle_resolve", ec.message().c_str());
// 		    if (ec.value() == boost::system::errc::operation_canceled) {
// 		      std::cout << "timed out";
// 		    } else {
// 		      std::cout << "waiting for server failed";

// 		    }
// 		    return;
// 		}


//	handle_connect(ec);


  } else {
    disconnect("handle_resolve", err.message().c_str());
  }
}


void
RosProxy::handle_connect(const boost::system::error_code &err)
{
  if (! err) {
//	wiat for hand shake
  	std::cout << "connecteed! \n";
  	serverInit=true;
  	read_from_server();
  }
  else {
    disconnect("handle_connect", err.message().c_str());
  }
}



void
RosProxy::read_from_server(){

  std::cout << "Waiting to read from  server! \n";
		 boost::asio::async_read(server_socket_, buff_s, boost::asio::transfer_at_least(1),
		 	boost::bind(&RosProxy::handle_server_reads, this,
		 				   boost::asio::placeholders::error)
		 	);

}

void
RosProxy::handle_server_reads(const boost::system::error_code &ec){


	if(!ec){
		//check if the socket is alive
		std::string s="";  
		std::ostringstream ss;
		ss << &buff_s;
		s = ss.str();

		//dispatcher_->write_to_client(s);
		write_to_socket(client_socket_,s);
		read_from_server();
	}
	else {
    disconnect("handle_server_reads", ec.message().c_str());
  }

}

// bool
// RosProxy::init_handshake(std::string msg){
	
// 	 	write_to_server(msg);

// //a temp solution for the no wait....

// //	 	boost::asio::read(client_server_, buff_s, boost::asio::transfer_at_least(1),
// //	 	boost::bind(&RosProxy::handle_handshak_req, this,
// //	 				   boost::asio::placeholders::error);
// //
// //	 	write_to_socket(client_socket_);
// //
// //	 	read_from_server_to_client();

// //check if success and return accourdingly
// 	 	return true;

// }



void RosProxy::write_to_server(std::string s){
	boost::system::error_code ec;

	size_t t =boost::asio::buffer_size(boost::asio::buffer(s));
	
	std::cout << "Writing to server!";
	std::cout << t;
	std::cout << "bytes \n!";
	std::cout << s;

	boost::asio::write(server_socket_,boost::asio::buffer(s) , ec);
  if(ec){
  		std::cout << "Failed to write! \n";
  		std::cout << ec.message().c_str();
  }

}

void
RosProxy::write_to_socket(boost::asio::ip::tcp::socket &socket , std::string s)
{
	boost::system::error_code ec;
	size_t t =boost::asio::buffer_size(boost::asio::buffer(s));
	

	std::cout << "Writing to client!";
	std::cout << t;
	std::cout << "bytes \n!";
	std::cout << s;

 	boost::asio::write(socket,boost::asio::buffer(s) , ec);
	if(ec){
	  		std::cout << "Failed to write! \n";
	  }
  
}

