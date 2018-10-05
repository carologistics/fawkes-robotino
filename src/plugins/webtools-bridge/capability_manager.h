
/***************************************************************************
 * capability_manager.h - Interface For Capability Managers
 *
 *  Created:  2016
 *  Copyright  21016 Mostafa Gomaa 
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

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
									, std::shared_ptr <WebSession> session)
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