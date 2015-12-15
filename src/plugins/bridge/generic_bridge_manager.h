#include <string>
#include <memory>

#include  "interfaces/ibridge_manager.h"
#include  "interfaces/ibridge_processor.h"//to be replaced with the actual processor not an interface
#include  "generic_bridge.h"

#include "subscribe.h"



class GenericBridgeManager : public  IbridgeManager, public std::enable_shared_from_this<GenericBridgeManager>
{
	public:
		GenericBridgeManager(std::shared_ptr<GenericBridge> bridge
			,std::shared_ptr<IBridgeProcessor> processor);
		~GenericBridgeManager();

		void register_operations();

		bool subscribe(std::string topic_name);


	protected:
		std::shared_ptr<IBridgeProcessor>		processor_;
		std::shared_ptr<GenericBridge> 			bridge_;
		
		std::shared_ptr<Subscribe>				subscribe_capability_;
};