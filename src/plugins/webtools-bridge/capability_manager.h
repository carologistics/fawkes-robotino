
#ifndef _CAPABILTY_MANAGER_H
#define _CAPABILTY_MANAGER_H

#include "bridge_processor.h"
#include <rapidjson/document.h>//todo:move to the cpps

#include <string>
#include <map>
#include <memory>

using rapidjson::Document;

class WebSession; 

class CapabilityManager
{
	public:

		CapabilityManager(std::string capability_name)
			: capability_name_(capability_name)
		{
		}
		
		~CapabilityManager()
		{
		}

		virtual void handle_message( rapidjson::Document &d 
									, std::shared_ptr <WebSession>)
		{}

		virtual void init() 
		{}

		virtual bool register_processor(std::shared_ptr <BridgeProcessor> processor)
		{return false;}


		std::string get_name()	{return capability_name_;}

	protected:
		typedef std::map< std::string , std::shared_ptr<BridgeProcessor> > 	ProcessorMap;
		
		ProcessorMap 	processores_;
		std::string 	capability_name_;
};

#endif