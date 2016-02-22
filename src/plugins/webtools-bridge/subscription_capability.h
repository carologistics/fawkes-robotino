#include <map>
#include <list>
#include <memory>
#include <utils/time/time.h>



namespace fawkes {
  class Clock;
  class Time;
  class Mutex;
 }
 
class WebSession;
class Subscription;

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
{	
	public:
		Subscription(std::string topic_name 
					, std::string processor_prefix 
					, fawkes::Clock *clock);

		~Subscription();
								//Object Operations
		void 					finalize();		//Implicitly calles deactivate
		void 					activate(); 	//Dont forget to set the right status
		void 					deactivate();	//Dont forget to set the right status
		void					subsume(std::shared_ptr <Subscription> another_subscription);

								//Processing Operations
		void 					publish();
		void					add_request( std::string id 		
											, std::string compression
											, unsigned int throttle_rate	
											, unsigned int queue_length 	
											, unsigned int fragment_size 	
											, std::shared_ptr<WebSession> session);
		void 					remove_request(std::string id 
												, std::shared_ptr<WebSession> session);
		void 					terminate_session_handler(std::shared_ptr<WebSession> session);

								//Status Checking Operations
		bool					is_active();
		bool 					empty();
		std::string 			get_topic_name();
		std::string 			get_processor_prefix();

	protected:
		virtual void 			finalize_impl();	//will be implicitly called from finialize()
		virtual void 			activate_impl(); 	//will be implicitly called from activate()
		virtual void 			deactivate_impl();	//will be implicitly called from deactive() 
		virtual std::string 	serialize(std::string op
											, std::string topic
											, std::string id);

		fawkes::Mutex 			*mutex_;
		
	private:
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



