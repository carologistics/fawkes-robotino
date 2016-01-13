#ifndef BRIDGE_MANAGER
#define BRIDGE_MANAGER

#include <string>
#include <memory>

#include  "interfaces/ibridge_manager.h"
#include  "interfaces/ibridge_processor.h"//to be replaced with the actual processor not an interface
#include  "generic_bridge.h"

#include "generic_capability_manager.h"
#include "web_sesison.h"



class BridgeManager :  public std::enable_shared_from_this<BridgeManager>
{
	public:
		BridgeManager();
		~BridgeManager();

		void incoming(std::string JsonMsg){
			Document  d;
			deserialize(d,jsonStr);
				//std::map  <std::string,std::string> request_details;
			if (d.HasMember("op"))
			{
			    std::string op_name= std::string(d["op"].GetString());
			    // std::string msg_id= std::string(d["id"].GetString());
			    std::cout <<"The op name is: "<<op_name<<std::endl;			     

				if(operation_manager_maping_.find(op_name)==operation_manager_maping_.end()){
					//throw exception or something
					 std::cout <<"No handler   for operation ("<<op_name<<")"<<std::endl;			     
					return;
				}

				capabilities_[op_name]->handle_message(d,std::string client_id);

			}


		}

		//May be here and may be just pass the session..decide later
		void outgoing(std::string jsonMsg, std::string client_id){
			clients_p[client_id]->send(jsonMsg)

		}

		void
			register_incoming_handler(){
			  //set the handlers to send and receive
			  web_session_->get_connection_ptr()->set_message_handler(bind(this::incoming,::_1,::_2));
			}


		bool register_operation(std::string operation_name,GenericCapabilityManager *cpm){
			if(capability_managers_.find(operation_name)==capability_managers_.end()){
				capability_managers_[operation_name]=cpm;
			}

		}

		void register_processor(std::shared_ptr<IBridgeProcessor> processor){
			for (std::map<std::string,GenericCapabilityManager>>::iterator it= operation_manager_maping_.begin()
					;it!=operation_manager_maping_.end(); ++it ){
				it->second->register_processor(processor);
			}
		}

		
	private:

		//std::shared_ptr<GenericBridge> 			bridge_;
		std::map <std::string,*GenericCapabilityManager>	 operation_cpm_maping_;
		std::map <std::string, web_sesison> clients_;			
		

};


#endif