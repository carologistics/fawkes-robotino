#include "interfaces/icapability.h"
#include "interfaces/ibridge_manager.h"

#include <rapidjson/document.h>
#include <iostream>
#include <map>
#include <memory>
#include <list>

using namespace rapidjson;

struct  details{
	std::string subscribtion_id;
	std::string msg_type;
   	double throttle_rate;
   	int queue_length;
   	std::string fragment_size;
   	// compression": compression
};



class Subscribtion{
	
	public:
		Subscribtion(std::string client_id, std::string topic);
		~Subscribtion();

		void subscribe(details *subscribe_args);
		void unsubscribe();

		void publish();

	private:
		std::string 			client_id_;
		std::string 			topic_;
		//handler 		 publish			find a way to register it
		std::list<details*> 		details_list_;

};


class Subscribe: public Icapability
{
	public:
		Subscribe(std::shared_ptr <IbridgeManager> manager);
		~Subscribe();

		void subscirbe(Document &d);
		void unsubscirbe(Document &d);

		void publish();

		void handle_message(Document &d);


	private:
		std::string client_id_;
		std::map <std::string,std::shared_ptr<Subscribtion> > topic_subscirbtions_;

};