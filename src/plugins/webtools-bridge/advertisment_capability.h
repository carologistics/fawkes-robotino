/***************************************************************************
 *  advertisment_capability.h - Advertisment Capabiliy Abstract Class
 *  Created: 2016 
 *  Copyright  2016 Mostafa Gomaa 
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


#ifndef __PLUGINS_ADVERTIS_CAPABILITY_H_
#define __PLUGINS_ADVERTIS_CAPABILITY_H_

#include <map>
#include <list>
#include <memory>

#include "callable.h"
#include "event_type.h"
#include "event_emitter.h"


namespace fawkes {
  class Mutex;
 }

class Advertisment;
class WebSession;
class EventEmitter;

class AdvertismentCapability
{
	public:
		virtual std::shared_ptr<Advertisment> advertise ( std::string topic_name 
															, std::string id 		
															, std::string type 	
														   	, std::shared_ptr<WebSession> session) = 0 ;

		virtual void 						unadvertise	( std::string id
															, std::shared_ptr<Advertisment> advertisment
															, std::shared_ptr<WebSession> session ) = 0 ;

		virtual void 						publish		( std::string id
															, bool latch
															, std::string msg_in_json //TODO:: figure out a clever way to keep track of msgs types and content without the need to have the info before hands
															, std::shared_ptr<Advertisment> advertisment
															, std::shared_ptr<WebSession> session ) = 0 ;
};


class Advertisment
:	public Callable
,	public EventEmitter
,	public std::enable_shared_from_this<Advertisment>
{
	public:
		Advertisment(std::string topic_name 
					, std::string processor_prefix);

		virtual ~Advertisment();
		
		void 					finalize();		//Implicitly calles deactivate
		void 					activate(); 	
		void 					deactivate();
		bool					is_active();
		bool 					empty();
		std::string 			get_topic_name();
		std::string 			get_processor_prefix();
		void					subsume(std::shared_ptr <Advertisment> another_advertisment);

		
		void					add_request( std::string id 		 	
											, std::shared_ptr<WebSession> session);
		void 					remove_request(std::string id 
												, std::shared_ptr<WebSession> session);

								//Callable impl. will be called when session emitts events
		void 					callback( EventType event_type , std::shared_ptr <EventEmitter> handler) ;

								//EventEmitter implementation (emitt event to SubCapManager)
		void					emitt_event (EventType event_type ) ;
	
	protected:
		virtual void 			finalize_impl();	//will be implicitly called from finialize()
		virtual void 			activate_impl(); 	//will be implicitly called from activate()
		virtual void 			deactivate_impl();	//will be implicitly called from deactive() 

		fawkes::Mutex 			*__mutex;
		
	protected:
		enum 			Status { ACTIVE, DORMANT };
		struct 			Request
						{
							Request(): id("") 
							{}
							~Request(){};
							std::string		id;
						};
		
		Status	 		active_status_;
		std::string 	topic_name_;
		std::string 	processor_prefix_;
	
		bool 			finalized; //set to true if it was Object was finilazed berfore
		
		std::map <std::shared_ptr<WebSession> , std::list<Request>>    			advertisments_; //maping of sessionTo requets list
		std::map <std::shared_ptr<WebSession> , std::list<Request>>::iterator 	it_advertisments_;
		std::list< Request >::iterator											it_requests_;
		
};

#endif