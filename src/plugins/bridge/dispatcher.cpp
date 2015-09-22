
#include "dispatecher.h"
#include "ros_proxy.h"
#include <boost/bind.hpp>
#include <exception> 
#include <boost/lexical_cast.hpp>

using namespace boost::asio;



dispatcher::dispatecher(unsigned short client_port,unsigned short client_port, unsigned short server_port):
		acceptor_(io_service_, ip::tcp::endpoint( ip::tcp::v6(), client_port))
		,client_socket_(io_service_)
		,client_port_(client_port)
		,ros_proxy_(new ros_proxy(server_port))
		{

		acceptor_.set_option(socket_base::reuse_address(true));
		start_accept();
		this->io_service_.run();
		
}



dispatcher::~dispatcher()
{
	boost::system::error_code err;
  	client_socket_.shutdown(ip::tcp::socket::shutdown_both, err);
  	client_socket_.close();
  	io_service_.stop();
  	
}
void dispatcher::disconnect(const char *where, const char *reason){

	std::cout << "Disconnected From ";
	std::cout << *where;
	std::cout << "Coz ";
	std::cout << *reason;

  boost::system::error_code ec;
  client_socket_.shutdown(ip::tcp::socket::shutdown_both, ec);
  client_socket_.close();
}

void
dispatcher::start_accept(){
	acceptor_.async_accept(client_socket_ ,boost::bind(&RosProxy::handle_accept, this,boost::asio::placeholders::error));
}

void 
dispatcher::handle_accept(const boost::system::error_code &ec){
	if(!ec){
		
		connected_to_rosbrindge=ros_proxy_->init_handshake(client_socket_);

		start_accept();
		start_dispatching();
	}
}

void
dispatcher::start_dispatching(){
	boost::asio::async_read(client_socket_, buff_c,boost::asio::transfer_at_least(1),
			boost::bind(&RosProxy::handle_client_reads, this,
						   boost::asio::placeholders::error)
			);
}


void
RosProxy::handle_client_reads(const boost::system::error_code &ec){

	if(!ec){

	std::string req="";  
	std::ostringstream ss;
	ss << &buff_c;
	req = ss.str();

	//find out if it is a jasonmsg and breakeit down
	//find where does it belonge

	//std::string reply="";  
	//call either the ROSwith or the fawkesAdapter and get a reply
	
	ros_proxy_->write_to_server(req);

	start_dispatching();
	}else {
    disconnect("handle_client_reads", ec.message().c_str());
  }

}
  
  
}
// void
// RosProxy::write_to_client(std::string reply)
// {
// 	size_t t =boost::asio::buffer_size(boost::asio::buffer(reply));

// 	std::cout << "Writing to client!";
// 	std::cout << t;
// 	std::cout << "bytes \n!";

//  	boost::asio::write(client_socket_,boost::asio::buffer(reply) , ec);
// 	if(ec){
// 	  		std::cout << "Failed to write! \n";
// 	  }
  
// }

int main(){

	dispatcher dispatcher_=new dispatcher(8080,9090);
}