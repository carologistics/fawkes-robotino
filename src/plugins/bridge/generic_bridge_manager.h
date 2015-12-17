#include <string>
#include <memory>

#include  "interfaces/ibridge_manager.h"
#include  "interfaces/ibridge_processor.h"//to be replaced with the actual processor not an interface
#include  "generic_bridge.h"

#include "subscribe.h"



class GenericBridgeManager : public  IbridgeManager, public std::enable_shared_from_this<GenericBridgeManager>
{
	public:
		GenericBridgeManager();
		~GenericBridgeManager();

		void register_bridge( std::shared_ptr<GenericBridge> bridge);
		void register_processor(std::shared_ptr<IBridgeProcessor> processor);

		bool subscribe(std::string topic_name);


	protected:
		void register_bridge_operations();

		std::shared_ptr<IBridgeProcessor> 		processor_;
		std::shared_ptr<GenericBridge> 			bridge_;
		
		std::shared_ptr<Subscribe>				subscribe_capability_;
};