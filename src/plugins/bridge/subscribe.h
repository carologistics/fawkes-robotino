
#include "interfaces/icapability.h"
#include "interfaces/ibridge_manager.h"

#include <rapidjson/document.h>
#include <iostream>
#include <map>
#include <memory>
#include <list>


using namespace rapidjson;

namespace fawkes {
  class Clock;
  class Time;
  class Mutex;
 }

//using namespace fawkes;

struct  details{
	details(): subscribtion_id(""),msg_type(""),throttle_rate(0),queue_length(1),fragment_size(0),compression("") {}
	std::string subscribtion_id;
	std::string msg_type;
   	int throttle_rate;
   	int queue_length;
   	int fragment_size;
   	std::string compression;
};


class Subscribtion
{
	
	public:
		Subscribtion(fawkes::Clock *clock, std::string client_id, std::string topic);
		~Subscribtion();

		void 		subscribe(details *subscribe_args);
		bool		unsubscribe(std::string subs_id);

		bool 		publish();

		std::string get_topic_name();
		int 		get_subscribtion_size();
	
	private:
		std::string 				topic_;
		std::string 				client_id_;
		//handler 		 publish			find a way to register it
		std::list<details*> 		details_list_;
		fawkes::Clock 				*clock_;
		fawkes::Time 				*last_published_time_;

		fawkes::Mutex				*sub_list_mutex_;
		fawkes::Mutex 				*time_mutex_;
};


class Subscribe
: public Icapability
{
	public:
		Subscribe(fawkes::Clock *clock, std::shared_ptr <IbridgeManager> manager);
		~Subscribe();

		void subscirbe(Document &d);
		bool unsubscribe(Document &d);

		bool publish();

		void handle_message(Document &d);

		std::map <std::string,std::shared_ptr<Subscribtion> > topic_subscirbtions_;

	private:
		fawkes::Clock 				*clock_;
		std::string client_id_;

		fawkes::Mutex				*random_mutex_;
		
};

