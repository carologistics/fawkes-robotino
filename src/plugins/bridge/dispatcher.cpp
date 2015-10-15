
#include "dispatcher.h"
#include <cassert>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/encodedstream.h"// AutoUTFInputStream
#include "rapidjson/memorystream.h"    
#include "rapidjson/error/en.h"
#include  "isession.h"



using namespace rapidjson;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;


Dispatcher::Dispatcher(websocketpp::lib::shared_ptr<Isession> web_s):
    rosbridge_started_(false),
    web_session_(web_s)
    {
  configure_web_session();

}


void
Dispatcher::web_configure_session(){
  //set the handlers to send and receive
  web_session_->get_connection_ptr()->set_message_handler(bind(&Dispatcher::web_on_message,this,::_1,::_2));

}



void
Dispatcher::web_on_message(connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg){
//Dispatches msg to one of the desired Bridges

  std::cout << "Dispatcheing....."<< std::endl;
  std::cout << msg->get_payload() << std::endl;

}


bool
Dispatcher::web_forward_message(std::string msg){

  //TODO:: add a check if the web_session is reachable
  //TODO:: catch exceptions
  return web_session_->send(msg);
}



// void
// Dispatcher::run(){
// 		init_rosbridge(rosbridge_ptr_);
// }


// void
// Dispatcher::init_rosbridge(ros_endpoint::ptr rosbridge_ptr){
    
//   rosbridge_ptr = websocketpp::lib::make_shared<ros_endpoint,std::owner_less<ros_endpointpmak>>();
//   int id =  rosbridge_ptr->connect("ws://localhost:9090");
//     if (id != -1) {
//         std::cout << "> Created connection with id " << id << std::endl;
//           rosbridge_started_=true;
//    } 
// }

void
Dispatcher::register_ros_endpoint(ros_endpoint::ptr rosbridge_ptr){
    
  rosbridge_ptr_=rosbridge_ptr;

  rosbridge_started_=true;
}

// void
// Dispatcher::register_web_endpoint( websocketpp::lib::shared_ptr<Iendpoint> ptr){
    
//   web_endpoint_ptr_=ptr;
// }


Dispatcher::~Dispatcher()
{
  	//delete rosbridge_;
 }

 bool
 Dispatcher::bridges_ready(){
 	return rosbridge_started_;
 }
  	

void
Dispatcher::dispatch_msg(std::string msg){

	std::cout<< "DISPATCHING" << std::endl;
	websocketpp::lib::error_code ec;
	rosbridge_ptr_->send(msg);
        if (ec) {
            std::cout << "> Error sending message: " << ec.message() << std::endl;
            return;
        }
}

void
Dispatcher::reply(std::string msg){

  std::cout<< "Replying" << std::endl;
  websocketpp::lib::error_code ec;
  //web_endpoint_ptr_->send(msg);
        if (ec) {
            std::cout << "> Error sending message: " << ec.message() << std::endl;
            return;
        }
}

// void
// Dispatcher::handle_client_reads(const boost::system::error_code &ec , size_t bytes_transferred){

// 	if(!ec){


// 	std::cout << "CLINET HANDLER \n";
	
// 	std::string s="";  
// 	std::ostringstream ss;
// 	ss << &buff_c;
// 	s = ss.str();
// 	//todo:choose the right rosProxy instance



// 	MemoryStream ms(s.c_str(),sizeof(s.c_str()));


// 	// std::cout << "is by \n";
// 	// std::cout << s; 
// 	// std::cout << "is by \n";

// 	// std::cout << is.rdbuf();

// 	// std::cout << "is by \n";
// 	// std::cout << bytes_transferred;
// 	// std::cout << "is sdfsdfy \n";
// 	// 	is>>s;


// 	// std::cout << s;
// 	// std::cout << "is sdfsdfy \n";
	
// 	// 	is>>s;


// 	// std::cout << s;
// 	// std::cout << "is sdfsdfy \n";

// 	// 	is>>s;


// 	// std::cout << s;
// 	// std::cout << "is sdfsdfy \n";

// 	// 	is>>s;



// 	//std::ostringstream ss;
// 	//eis << &buff_c;
// 	//s = eis.str();
// 	//todo:choose the right rosProxy instance


// 	//this waiting for server has to be replaced with a better solution
// 	do{
// 	std::cout << "waiting for rosbridge proxy to come alive \n";
// 	}
// 	while(!rosProxy_->check_rosBridge_alive());
// 	rosProxy_->process_req(s);


		
// 	AutoUTFInputStream<unsigned, MemoryStream> eis(ms);

// 	 Document d;
// 	 if(!d.ParseStream<0, AutoUTF<unsigned> >(eis).HasParseError()){
// 		std::cout << "JSON: parsed! \n";
// 		if(d.IsObject()){
// 			std::cout << "JSON: Object! \n";
// 			if(d.HasMember("op")){
// 			std::cout << "JSON: op is there! \n";
// 			}
// 			else{std::cout << "NOT Member! \n";}

		
// 		}
// 		else
// 			std::cout << "NOT object \n!";

// }
// else{
//  fprintf(stderr, "\nError(offset %u): %s\n", 
//         (unsigned)d.GetErrorOffset(),
//         GetParseError_En(d.GetParseError()));
//     // ...
// 	}

// 	start_dispatching();
// 	}else {
//     //disconnect("handle_client_reads", ec.message().c_str());
//   }

 //}
  
 
// int main(){
// 	Dispatcher *dispatcher=new Dispatcher();
// 	dispatcher->init();
// }
