
#include "dispatcher.h"
#include "ros_proxy.h"
#include <boost/bind.hpp>
#include <exception> 
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


using namespace boost::asio;





Dispatcher::Dispatcher(unsigned short client_port, unsigned short server_port):
		acceptor_(io_service_, ip::tcp::endpoint( ip::tcp::v6(), client_port))
		,client_port_(client_port)
		,server_port_(server_port)
		{

		acceptor_.set_option(socket_base::reuse_address(true));
		//boost::shared_ptr<boost::asio::io_service> io_service_ptr= boost::allocate_shared(io_service_);
		rosProxy_=new RosProxy(io_service_, server_port_);
		start_accept();		
		this->io_service_.run();

		
}



Dispatcher::~Dispatcher()
{
	boost::system::error_code err;
  	//client_socket_.shutdown(ip::tcp::socket::shutdown_both, err);
  	//client_socket_.close();
  	io_service_.stop();
  	delete rosProxy_;
  	
}
void Dispatcher::disconnect(const char *where, const char *reason){

	std::cout << "Disconnected From ";
	std::cout << *where;
	std::cout << "Coz ";
	std::cout << *reason;

  boost::system::error_code ec;
 // client_socket_.shutdown(ip::tcp::socket::shutdown_both, ec);
 // client_socket_.close();
}

void
Dispatcher::start_accept(){

	
	acceptor_.async_accept(rosProxy_->client_socket_,boost::bind(&Dispatcher::handle_accept, this,boost::asio::placeholders::error));
}

void 
Dispatcher::handle_accept(const boost::system::error_code &ec){
	if(!ec){

				
		boost::system::error_code ec;
		//intiat the handshake with the rosBridge
	//	boost::asio::read(client_socket_, buff_c, boost::asio::transfer_at_least(1),ec);
		


		//serverInit=rosProxy_->init_handshake(s);

		start_dispatching();
		start_accept();

	}
}

void
Dispatcher::start_dispatching(){

	boost::asio::async_read(rosProxy_->client_socket_, buff_c,boost::asio::transfer_at_least(1),
			boost::bind(&Dispatcher::handle_client_reads, this,
						   boost::asio::placeholders::error)
			);
}


void
Dispatcher::handle_client_reads(const boost::system::error_code &ec){

	if(!ec){

	//ALL THE JASON STUFFaf
	std::string s="";  
	std::ostringstream ss;
	ss << &buff_c;
	s = ss.str();

	//todo:choose the right rosProxy instance

	rosProxy_->write_to_server(s);

	start_dispatching();
	}else {
    disconnect("handle_client_reads", ec.message().c_str());
  }

}
  
// void
// Dispatcher::write_to_client(std::string s)
// {
// 	boost::system::error_code ec;
 	
// 	size_t t =boost::asio::buffer_size(boost::asio::buffer(s));
	

// 	std::cout << "Writing to client!";
// 	std::cout << t;
// 	std::cout << "bytes \n!";

//  	boost::asio::write(*client_socket_ptr,boost::asio::buffer(s) , ec);
// 	if(ec){
// 	  		std::cout << "Failed to write! \n";
// 	  }
  
// }
  
 
int main(){
	Dispatcher *dispatcher=new Dispatcher(8080,9090);
}
