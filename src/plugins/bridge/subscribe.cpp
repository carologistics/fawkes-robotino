#include "subscribe.h"


#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include <cstdio>

//later::dont forget to propagae the client_id from bridge till here
		Subscribe::Subscribe(std::shared_ptr <IbridgeManager> manager )
		:Icapability(manager )
		{
			client_id_="";
		}

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

			std::cout<< " TO" << topic_name << std::endl;

			if(topic_subscirbtions_.find(topic_name)==topic_subscirbtions_.end()){
				topic_subscirbtions_[topic_name]= std::make_shared<Subscribtion>(client_id_,topic_name);
			}

			    // Iterating object members
    			static const char* kTypeNames[] = { "Null", "False", "True", "Object", "Array", "String", "Number" };
		    	for (Value::ConstMemberIterator itr = d.MemberBegin(); itr != d.MemberEnd(); ++itr)
        		printf("Type of member %s is %s\n", itr->name.GetString(), kTypeNames[itr->value.GetType()]);


        	///todo:: make sure the members are there before quering them
			details subscribe_args;
			subscribe_args.subscribtion_id=std::string(d["id"].GetString());
			subscribe_args.msg_type=std::string(d["type"].GetString());
			subscribe_args.throttle_rate=d["throttle_rate"].GetDouble();
			subscribe_args.queue_length=d["queue_length"].GetInt();
			// // subscribe_args.fragment_size=std::string(d["fragment_size"].GetString());

			topic_subscirbtions_[topic_name]->subscribe(&subscribe_args);

		}

		void
		Subscribe::publish(){}


		Subscribtion::Subscribtion(std::string client_id, std::string topic)
		:client_id_(client_id)
		,topic_(topic)
		{}

		Subscribtion::~Subscribtion(){}

		void
		Subscribtion::subscribe(details *subscirbe_args){
  			std::cout <<"REACHED THE SUBSCRIBTION STAGE WITH "<<topic_<<std::endl;
			details_list_.push_back(subscirbe_args);
		}
