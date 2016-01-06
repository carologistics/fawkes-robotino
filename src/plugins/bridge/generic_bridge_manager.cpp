#include  "generic_bridge_manager.h"

		GenericBridgeManager::GenericBridgeManager(fawkes::Clock *clock)
		:bridge_inited_(false),
		processor_inited_(false)
		{
			clock_=clock;
		}

		GenericBridgeManager::~GenericBridgeManager(){}


		void GenericBridgeManager::register_bridge(std::shared_ptr<GenericBridge> bridge){
			bridge_=bridge;
			register_bridge_operations();
			bridge_inited_=true;
		}

		void GenericBridgeManager::register_processor(std::shared_ptr<IBridgeProcessor> processor){
			processor_=processor;
			processor_inited_=true;
		}
		


		void 
		GenericBridgeManager::register_bridge_operations(){
			subscribe_capability_=std::make_shared<Subscribe>(clock_,this->shared_from_this());
			bridge_->register_operation("subscribe",subscribe_capability_);
		}


		//-------Propagating to processor and Handling of publishing
		bool 
		GenericBridgeManager::subscribe(std::string topic_name){
			bool result = processor_->subscribe(topic_name);// TODO:: catch exceptions
			if(result){
				std::cout<< "Subscribed to topic:"<<topic_name<<std::endl;
				std::string jsonStr= processor_->publish_topic(topic_name,"");
				bridge_->outgoing(jsonStr);
			}
			else
			{
				std::cout<<"Topic does not exist in distenation";
				//unsubscribe it from the bridge
			}	
			return result;
		}

		bool GenericBridgeManager::loop(){
			std::cout<<"LOOP"<<std::endl;
			if(bridge_inited_ && processor_inited_){
				std::cout<<"Publishing......."<<std::endl;
				subscribe_capability_->publish();
				return true;
			}
			return false;
		}
		



