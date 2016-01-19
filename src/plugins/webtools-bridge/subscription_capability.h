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

class Subscription
{	
	public:
		Subscription(std::string topic_name 
					, std::string processor_prefix 
					, fawkes::Clock *clock);

		~Subscription();// finalize oper and listeners interfaces 
		
		std::string get_topic_name();
		std::string get_processor_prefix();

		void 	activate();
		void 	deactivate();
		void	append(std::shared_ptr <Subscription> another_subscription);
		bool 	empty();

		void 	finalize();

		// void	add_Subscription_request(rapidjson::Document &d
		// 								,std::shared_ptr <WebSession> session);
		// void	remove_Subscription_request(rapidjson::Document &d
		// 									,std::shared_ptr <WebSession> session);

	protected:
		void 	publish(std::string json_str);

	private:
	
		struct SubscriptionRequest{
			SubscriptionRequest()
				:	id("") , compression("")
				,	throttle_rate(0) ,queue_length(1) , fragment_size(0)	
			{
			}
			~SubscriptionRequest()
			{
				delete last_published_time;
			}

			std::string		id;
		   	std::string 	compression;
		   	unsigned int 	throttle_rate;
		   	unsigned int 	queue_length;
		   	unsigned int 	fragment_size;

		 	fawkes::Time	*last_published_time;
		};

		typedef std::list<SubscriptionRequest> RequestList;
		
		std::map <std::shared_ptr<WebSession> , RequestList>    subscribers_; //maping of sessionTo requets list
		std::string 											topic_name_;
		std::string 											processor_prefix_;


		bool static compare_throttle_rate(SubscriptionRequest first, SubscriptionRequest second);
		fawkes::Clock 				*clock_;

		enum status { ACTIVE, DORMANT };
		// fawkes::Mutex				*sub_list_mutex_;
		// fawkes::Mutex 				*time_mutex_;
};


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

