
#include <cassert>
#include <exception>
#include <iostream>
#include <sstream>


#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
// #include "rapidjson/encodedstream.h"// AutoUTFInputStream
// #include "rapidjson/memorystream.h"    
#include "rapidjson/error/en.h"

#include "isession.h"
#include "dispatcher.h"


using namespace rapidjson;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;


Dispatcher::Dispatcher(websocketpp::lib::shared_ptr<Isession> web_s)
    : rosbridge_started_(false)
    , web_session_(web_s){}

Dispatcher::~Dispatcher()
{
  	//delete rosbridge_;
 }

void Dispatcher::start(){
  register_bridges();
  //TODO::make sure it is initialized before proceeding
  web_register_handler();
}

void
Dispatcher::register_bridges(){   
  bridges_[bridgeType::ROS_BRIDGE] = websocketpp::lib::make_shared<ros_proxy>(this->shared_from_this());
  rosbridge_started_= bridges_[bridgeType::ROS_BRIDGE]->init();

  //Add The Fawkes Bridge HERE
}


void
Dispatcher::web_register_handler(){
  //set the handlers to send and receive
  web_session_->get_connection_ptr()->set_message_handler(bind(&Dispatcher::web_on_message,this,::_1,::_2));
}


//the single enrty point to the Birdges
void
Dispatcher::web_on_message(connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr web_msg){
//Dispatches msg to one of the desired Bridges

  std::cout << "Dispatcheing....."<< std::endl;
  std::cout << web_msg->get_payload() << std::endl;

  std::string jsonString = web_msg->get_payload();
  bridges_ [ dispatch(jsonString)] -> process_request(jsonString);
}



bool
Dispatcher::web_forward_message(std::string msg){

  //TODO:: add a check if the web_session is reachable
  //TODO:: catch exceptions
  return web_session_->send(msg);
}


bridgeType
Dispatcher::dispatch(std::string msg){
  const char* json = msg.c_str();
  Document d;
  d.Parse(json);

  if (d.Parse(json).HasParseError()) {
    std::cout<< GetParseError_En(d.GetParseError());
  }
  
  if (d.HasMember("op"))
   {
    std::cout << "------->FOUND OP....."<<d["op"].GetString()<< std::endl;

    if (std::string(d["topic"].GetString()).find("blackboard") != std::string::npos)
    {
      std::cout<< "That should not happen"<<std::endl;
      return bridgeType::BLACKBOARD_BRDIGE;
    }
  }
  return bridgeType::ROS_BRIDGE;
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
