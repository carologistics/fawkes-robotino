
#ifndef _CAPABILTY_MANAGER_H
#define _CAPABILTY_MANAGER_H

#include "bridge_processor.h"

#include <string>
#include <map>
#include <memory>


class WebSession; 

class CapabilityManager
{
	public:

		CapabilityManager(std::string capability_name)
		:	capability_name_(capability_name)
		{
		}
		
		~CapabilityManager()
		{
		}

		virtual void handle_message(rapidjson::Document &d , std::shared_ptr <WebSession>) = 0;

		virtual bool register_processor(std::shared_ptr <BridgeProcessor> processor) = 0;

		std::string get_name(){
			return capability_name_;
		}

	protected:
		std::string 														capability_name_;
		std::map< std::string , std::shared_ptr<BridgeProcessor> > 			processores_;
};

#endif