#include <map>
#include <list>
#include <memory>
#include <utils/time/time.h>

#include "callable.h"
#include "event_type.h"
#include "event_emitter.h"




namespace fawkes {
  class Clock;
  class Time;
  class Mutex;
 }

class Subscription;
class WebSession;
class EventEmitter;

//=================================   SubscribeCapability  ===================================
class SubscriptionCapability
{
	public:
		virtual std::shared_ptr<Subscription> 	subscribe 	( std::string topic_name 
															, std::string id 		
															, std::string compression
															, unsigned int throttle_rate	
															, unsigned int queue_length 	
															, unsigned int fragment_size 	
														   	, std::shared_ptr<WebSession> session) = 0 ;

		virtual void 							unsubscribe	( std::string id
															, std::shared_ptr<Subscription> 
															, std::shared_ptr<WebSession> session ) = 0 ;
};


//=================================   Subscription   ===================================
class Subscription
:	public Callable
,	public EventEmitter
,	public std::enable_shared_from_this<Subscription>
{	
	public:
		Subscription(std::string topic_name 
					, std::string processor_prefix 
					, fawkes::Clock *clock);

		virtual ~Subscription();
		
		void 					finalize();		//Implicitly calles deactivate
		void 					activate(); 	
		void 					deactivate();
		bool					is_active();
		bool 					empty();
		std::string 			get_topic_name();
		std::string 			get_processor_prefix();
		void					subsume(std::shared_ptr <Subscription> another_subscription);

		
		void					add_request( std::string id 		
											, std::string compression
											, unsigned int throttle_rate	
											, unsigned int queue_length 	
											, unsigned int fragment_size 	
											, std::shared_ptr<WebSession> session);
		void 					remove_request(std::string id 
												, std::shared_ptr<WebSession> session);

								//Callable impl. will be called when session emitts events
		void 					callback( EventType event_type , std::shared_ptr <EventEmitter> handler) ;

								//EventEmitter implementation (emitt event to SubCapManager)
		void					emitt_event (EventType event_type ) ;
		
		void 					publish();

	protected:
		virtual void 			finalize_impl();	//will be implicitly called from finialize()
		virtual void 			activate_impl(); 	//will be implicitly called from activate()
		virtual void 			deactivate_impl();	//will be implicitly called from deactive() 
		virtual std::string 	serialize(std::string op
											, std::string topic
											, std::string id);

		fawkes::Mutex 			*mutex_;
		
	protected:
		void 					add_new_session(std::shared_ptr<WebSession> session);	
		void 					remove_session(std::shared_ptr<WebSession> session);


		enum 			Status { ACTIVE, DORMANT };
		struct 			Request
						{
							Request(): id("") ,compression("")	,throttle_rate(0) ,queue_length(1) ,fragment_size(0)
							{}
							std::string		id;
						   	std::string 	compression;
						   	unsigned int 	throttle_rate;
						   	unsigned int 	queue_length;
						   	unsigned int 	fragment_size;
						 	std::shared_ptr<fawkes::Time>	last_published_time;
						};
		


		Status	 		active_status_;
		std::string 	topic_name_;
		std::string 	processor_prefix_;
		fawkes::Clock	*clock_;
		bool 			finalized; //set to true if it was Object was finilazed berfore
		
		std::map <std::shared_ptr<WebSession> , std::list<Request>>    			subscriptions_; //maping of sessionTo requets list
		std::map <std::shared_ptr<WebSession> , std::list<Request>>::iterator 	it_subscriptions_;
		std::list< Request >::iterator											it_requests_;
		
		bool static compare_throttle_rate(Request first, Request second);

		// fawkes::Mutex				*sub_list_mutex_;
		// fawkes::Mutex 				*time_mutex_;

};



