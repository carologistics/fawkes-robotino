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

//=================================   Subscription  ===================================
class Subscription
{	
	public:
		Subscription(std::string topic_name 
					, std::string processor_prefix 
					, fawkes::Clock *clock);

		~Subscription();
		
		std::string get_topic_name();
		std::string get_processor_prefix();
		
		void		subsume(std::shared_ptr <Subscription> another_subscription);
		bool 		empty();

		void 		activate(); 	//Dont forget to set the right status
		void 		deactivate();	//Dont forget to set the right status
		void 		finalize();		//Implicitly calles deactivate

		virtual std::string 	serialize(std::string op
								, std::string topic
								, std::string id);

		void add_Subscription_request( std::string id 		
									, std::string compression
									, unsigned int throttle_rate	
									, unsigned int queue_length 	
									, unsigned int fragment_size 	
								   	, std::shared_ptr<WebSession> session);

	
		void remove_Subscription_request(std::string id
		 								,std::shared_ptr <WebSession> session);

		void on_terminate_session(std::shared_ptr<WebSession> session);

	private:
		void register_session_handlers(std::shared_ptr<WebSession> session);
			
		bool finalized; //set to true if it was Object was finilazed berfore
	
	protected:

		struct SubscriptionRequest{
			SubscriptionRequest()
				:	id("") , compression("")
				,	throttle_rate(0) ,queue_length(1) , fragment_size(0)	
			{
			}

			std::string		id;
		   	std::string 	compression;
		   	unsigned int 	throttle_rate;
		   	unsigned int 	queue_length;
		   	unsigned int 	fragment_size;

		 	std::shared_ptr<fawkes::Time>	last_published_time;
		};
		
		typedef std::list<SubscriptionRequest> 	RequestList;
		enum Status { ACTIVE, DORMANT };

	

		void publish();

		virtual void activate_impl(); 	//will be implicitly called from activate()
		virtual void deactivate_impl();	//will be implicitly called from deactive() and finalize()
		virtual void finalize_impl();	//will be implicitly called from finialize()
	
		std::string 		 	topic_name_;
		std::string 			processor_prefix_;
		fawkes::Clock			*clock_;
		
		Status	 												active_status_;

		std::map <std::shared_ptr<WebSession> , RequestList>    subscribers_; //maping of sessionTo requets list

		bool static compare_throttle_rate(SubscriptionRequest first, SubscriptionRequest second);

		// fawkes::Mutex				*sub_list_mutex_;
		// fawkes::Mutex 				*time_mutex_;

};



//=================================   Subscribe  ===================================
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

