#include "subscribe.h"

//later::dont forget to propagae the client_id from bridge till here
		Subscribe::Subscribe(){}

		Subscribe::~Subscribe(){}
		
		void 		
		Subscribe::handle_message(Document &d){
			std::string op_name= std::string(d["op"].GetString());

			if (op_name=="subscribe")
				subscirbe(d);
			else
				std::cout<< "No operation with this name in this capability"<<std::endl;
		}

		void 
		Subscribe::subscirbe(Document &d){

			std::cout<< "Subscribing";

			std::string topic_name= std::string(d["topic"].GetString());

			std::cout<< "TO" << topic_name << std::endl;

			if(topic_subscirbtions_.find(topic_name)==topic_subscirbtions_.end()){
				topic_subscirbtions_[topic_name]=new Subscribtion(client_id_,topic_name);
			}

			details subscribe_args;
			subscribe_args.subscribtion_id=std::string(d["sid"].GetString());
			subscribe_args.msg_type=std::string(d["type"].GetString());
			subscribe_args.throttle_rate=std::string(d["throttle_rate"].GetString());
			subscribe_args.queue_length=std::string(d["queue_length"].GetString());
			subscribe_args.fragment_size=std::string(d["fragment_size"].GetString());


			topic_subscirbtions_[topic_name]->subscribe(subscribe_args);

		}


		Subscribtion::Subscribtion(std::string client_id, std::string topic)
		:client_id_(client_id)
		,topic_(topic)
		{}

		Subscribtion::~Subscribtion(){}

		void
		Subscribtion::subscribe(details subscirbe_args){
 			
 			std::cout <<"REACHED THE SUBSCRIBTION STAGE WITH "<<topic_<<std::endl;
			// details_list_.push_back(subscirbe_args);

		}
