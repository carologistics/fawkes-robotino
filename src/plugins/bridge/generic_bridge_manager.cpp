#include  "generic_bridge_manager.h"

		GenericBridgeManager::GenericBridgeManager()
		{}

		GenericBridgeManager::~GenericBridgeManager(){}


		void GenericBridgeManager::register_bridge(std::shared_ptr<GenericBridge> bridge){
			bridge_=bridge;
			register_bridge_operations();
		}

		void GenericBridgeManager::register_processor(std::shared_ptr<IBridgeProcessor> processor){
			processor_=processor;
		}
		


		void 
		GenericBridgeManager::register_bridge_operations(){
			subscribe_capability_=std::make_shared<Subscribe>(this->shared_from_this());
			bridge_->register_operation("subscribe",subscribe_capability_);
		}


		//-------Propagating to processor and Handling of publishing
		bool 
		GenericBridgeManager::subscribe(std::string topic_name){
			bool result = processor_->subscribe(topic_name);
			if(result){
				std::cout<< "Subscribed to topic:"<<topic_name<<std::endl;
				std::string jsonStr= processor_->read_single_topic(topic_name);
				bridge_->outgoing(jsonStr);
			}
			else
			{
				std::cout<<"Topic does not exist in distenation";
			}	
			return result;
		}




