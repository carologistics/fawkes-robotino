#include  "generic_bridge_manager.h"

		GenericBridgeManager::GenericBridgeManager(std::shared_ptr<GenericBridge> bridge
			,std::shared_ptr<IBridgeProcessor> processor)
		:processor_(processor)
		,bridge_(bridge)
		{
			subscribe_capability_=std::make_shared<Subscribe>(this->shared_from_this());
			bridge->register_operation("subscribe",subscribe_capability_);

		}

		GenericBridgeManager::~GenericBridgeManager(){}


		
		void 
		GenericBridgeManager::register_operations(){

			//Move all operation registration here
		}


		bool 
		GenericBridgeManager::subscribe(std::string topic_name){
			bool result = processor_->subscribe(topic_name);
			if(result){
				std::cout<< "Subscribed to topic:"<<topic_name<<std::endl;
			}
			else
			{
				std::cout<<"Topic does not exist in distenation";
			}	
			return result;
		}




