#include "interfaces/icapability_manager.h"
#include "subscribe.h"


//	Couldnt this be done with templetes

class SubscribeCapabilityManager
: public generic_capability_manger
{

public: 
		 SubscribeCapabilityManager()
		 :GenericCapabilityManager("subscribe")
		 {}

 		~SubscribeCapabilityManager();

 		bool register_processor(std::shared_ptr <IBridgeProcessor> processor ){

 			////through the dynamic casting it knows if the capability is extended in the processor
 		}

 		void handle_message(Document &d, std::string client_id){

 			//first decides which Source to call ..longest match ..
 			//Q .. Do i have to look within each processor or is it enough with the map
 				std::string source_name= "blackboard";
 		
 		//decides which opreation it should call

 				std::string op_name= std::string(d["op"].GetString());

 				if(op_name=="subscribe")
 				 processores_[source_name].subscribe(Document &d, std::string client_id);
 				else if (op_name=="unsubscribe")
 					processores_[source_name].unsubscribe(Document &d, std::string client_id);

 		}



};