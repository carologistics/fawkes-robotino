
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



using namespace rapidjson;

Dispatcher::Dispatcher():rosbridge_started_(false) {}



void
Dispatcher::run(){
		init_rosbridge(rosbridge_ptr_);
}


void
Dispatcher::init_rosbridge(ros_endpoint::ptr rosbridge_ptr){
    
  rosbridge_ptr = websocketpp::lib::make_shared<ros_endpoint>();
  int id =  rosbridge_ptr->connect("ws://localhost:9090");
    if (id != -1) {
        std::cout << "> Created connection with id " << id << std::endl;
          rosbridge_started_=true;
   }
}

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
	rosbridge_ptr_->send(0,msg);
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
