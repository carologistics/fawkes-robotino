
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
 }

//using namespace fawkes;

struct  details{
	std::string subscribtion_id;
	std::string msg_type;
   	double throttle_rate;
   	int queue_length;
   	std::string fragment_size;
   	// compression": compression
};


class Subscribtion
{
	
	public:
		Subscribtion(fawkes::Clock *clock, std::string client_id, std::string topic);
		~Subscribtion();

		void subscribe(details *subscribe_args);
		void unsubscribe();

		bool publish();

	private:
		std::string 				client_id_;
		std::string 				topic_;
		//handler 		 publish			find a way to register it
		std::list<details*> 		details_list_;
		fawkes::Clock 				*clock_;
		fawkes::Time 				*last_published_time_;
};


class Subscribe
: public Icapability
{
	public:
		Subscribe(fawkes::Clock *clock, std::shared_ptr <IbridgeManager> manager);
		~Subscribe();

		void subscirbe(Document &d);
		void unsubscirbe(Document &d);

		bool publish();

		void handle_message(Document &d);

		std::map <std::string,std::shared_ptr<Subscribtion> > topic_subscirbtions_;

	private:
		fawkes::Clock 				*clock_;
		std::string client_id_;

};

