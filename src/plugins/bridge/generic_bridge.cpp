
#include "generic_bridge.h"

	
	GenericBridge::GenericBridge(std::shared_ptr<Idispatcher> dispatcher)
	:dispatcher_(dispatcher)
	{

	}

	GenericBridge::~GenericBridge(){}

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
		//nothing I can think of now
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
	}
