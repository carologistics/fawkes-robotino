
#include "dispatcher.h"
#include "ros_proxy.h"
#include <boost/bind.hpp>
#include <exception> 
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <cassert>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>


#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

using namespace boost::asio;
using namespace rapidjson;




Dispatcher::Dispatcher(unsigned short client_port, unsigned short server_port):
		acceptor_(io_service_, ip::tcp::endpoint( ip::tcp::v6(), client_port))
		,client_port_(client_port)
		,server_port_(server_port)
		{
		acceptor_.set_option(socket_base::reuse_address(true));
		start_accept();
		this->io_service_.run();
}



Dispatcher::~Dispatcher()
{
	boost::system::error_code err;
  	io_service_.stop();
  	delete rosProxy_;
  	
}

void
Dispatcher::start_accept(){
	//this is wrong ...this makes a new instance in the same var every time someone tries to connect
	//we want to have one of this mapping instance every
	rosProxy_=new RosProxy(io_service_, server_port_); 
	rosProxy_->start_server();
	acceptor_.async_accept(rosProxy_->client_socket_,boost::bind(&Dispatcher::handle_accept, this,boost::asio::placeholders::error));
}


void 
Dispatcher::handle_accept(const boost::system::error_code &ec){
	if(!ec){		
		boost::system::error_code ec;

		start_dispatching();

		//Before you start accept again you have to make sure that it would replace the rosProxy instance
		//start_accept();
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

	
	std::string s="";  
	std::ostringstream ss;
	ss << &buff_c;
	s = ss.str();
	//todo:choose the right rosProxy instance


	//this waiting for server has to be replaced with a better solution
	do{
	std::cout << "waiting for rosbridge proxy to come alive \n";
	}
	while(!rosProxy_->check_rosBridge_alive());
	rosProxy_->process_req(s);


	Document d;
	if (!d.Parse<0>(s.c_str()).HasParseError())
	{
		std::cout << "JSON: parsed! \n";
		if(d.IsObject()){
			std::cout << "JSON: Object! \n";
			if(d.HasMember("op")){
			std::cout << "JSON: op is there! \n";
			}
			else{std::cout << "NOPE!";}

		
		}
		else
			std::cout << "NOPE!";


	}
		else
			std::cout << "NOPE!";
    
	

	start_dispatching();
	}else {
    //disconnect("handle_client_reads", ec.message().c_str());
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
