
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

		fawkes::Mutex 			*mutex_;
		
	protected:
		enum 			Status { ACTIVE, DORMANT };
		struct 			Request
						{
							Request(): id("") 
							{}
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