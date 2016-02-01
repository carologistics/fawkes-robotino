#ifndef _BRIDGE_MANAGER_H
#define _BRIDGE_MANAGER_H

#include <string>
#include <memory>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>//TODO:: move to the cpp


#include "bridge_processor.h"
#include "capability_manager.h"
#include "web_session.h"


/*TODDO:Change the name of this class sugesgtions
	* CapabiltiesDispatcher
	* BridgeMother
	* 
*/
class BridgeManager 
//:  public std::enable_shared_from_this<BridgeManager>
{
	public:
		BridgeManager();
		~BridgeManager();

		void incoming(std::string JsonMsg ,std::shared_ptr <WebSession> session);

		void outgoing(std::string jsonMsg , std::string client_id);

		bool deserialize(std::string jsonSt , rapidjson::Document &d);

		bool register_operation_handler(std::string operation_name , std::shared_ptr <CapabilityManager> cpm);

		bool register_processor(std::shared_ptr <BridgeProcessor> processor);
		
	private:
		std::map< std::string , std::shared_ptr <CapabilityManager> >	 		operation_cpm_map_;
		//std::map <std::string, WebSessison> 									clients_;			
};


#endif