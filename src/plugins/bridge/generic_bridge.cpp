
#include "generic_bridge.h"

	
	GenericBridge::GenericBridge(std::shared_ptr<Idispatcher> dispatcher ,std::string target_prefix)
	:target_prefix_(target_prefix)
	,dispatcher_(dispatcher)
	
	{}

	GenericBridge::~GenericBridge(){}

	void 
	GenericBridge::incoming(std::string jsonStr){
			Document  d;
			deserialize(d,jsonStr);
				//std::map  <std::string,std::string> request_details;
			if (d.HasMember("op"))
			{
			    std::string op_name= std::string(d["op"].GetString());
			    // std::string msg_id= std::string(d["id"].GetString());
			    std::cout <<"The op name is: "<<op_name<<std::endl;			     
				   
			    std::string topic_fullname=std::string(d["topic"].GetString());

			    std::size_t pos = topic_fullname.find(target_prefix_);
			     if (pos != std::string::npos)
			    {
			    	 std::string topic_name= topic_fullname.erase(pos 
			    	 	, target_prefix_.length()+1);

			    	 d["topic"].SetString(StringRef(topic_name.c_str()));

			    	 std::cout <<"The topic name is: "<<topic_name<<std::endl;			     
			    }

				if(capabilities_.find(op_name)==capabilities_.end()){
					//throw exception or something
					 std::cout <<"No handler registerd for operation ("<<op_name<<")"<<std::endl;			     
					return;
				}

				capabilities_[op_name]->handle_message(d);

			}

		}

	//forwards the outgoing msg to the web client 
	void 
	GenericBridge::outgoing(std::string jsonStr){
		dispatcher_->send_to_web(jsonStr);
	}



	bool 
	GenericBridge::deserialize(Document &d, std::string jsonStr){
		const char* json = jsonStr.c_str();
		d.Parse(json);

		if (d.Parse(json).HasParseError()) {
			std::cout<< GetParseError_En(d.GetParseError());
			return false;
		}

		return true;
	}

	void 
	GenericBridge::serialize(std::string jsonStr){}


	//overriding from IBridge
	bool 
	GenericBridge::init(){
			//TODO//replace with correct instance of bridgeManager not a new one
			return true;
		}
	void 
	GenericBridge::process_request(std::string jsonStr){
		incoming(jsonStr);
	}


	void
	GenericBridge::register_operation(std::string op_name
		, std::shared_ptr<Icapability> handler){
		
		capabilities_[op_name]=handler;
		std::cout<< "registered"<<std::endl;
			


	}
