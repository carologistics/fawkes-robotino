
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
			, initialized_(false)
			, finalized_(false)
		{
		}
		
		~CapabilityManager()
		{
		}

		virtual void handle_message( rapidjson::Document &d 
									, std::shared_ptr <WebSession>)
		{}

		virtual void init() 
		{
			if(!initialized_)
			{
				initialized_ = true ; 
			}

		}
		
		virtual void finalize()
		{
			if(!finalized_)
			{
				while (!processores_.empty())
				{
					processores_.begin()-> second ->finalize();
					processores_.erase(processores_.begin());
				}

				finalized_=true;
			}
		}

		virtual bool register_processor(std::shared_ptr <BridgeProcessor> processor)
		{return false;}


		std::string get_name()	{return capability_name_;}

	protected:
		typedef std::map< std::string , std::shared_ptr<BridgeProcessor> > 	ProcessorMap;
		
		ProcessorMap 	processores_;
		std::string 	capability_name_;
		bool initialized_;
		bool finalized_;
};

#endif