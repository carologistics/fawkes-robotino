#include "subscribe.h"

#include <utils/time/time.h>
#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include <cstdio>



using namespace fawkes;

//later::dont forget to propagae the client_id from bridge till here
		Subscribe::Subscribe(fawkes::Clock *clock, std::shared_ptr <IbridgeManager> manager )
		:Icapability(manager)
		{
			clock_=clock;
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
					topic_subscirbtions_[topic_name]= std::make_shared<Subscribtion>(clock_,client_id_,topic_name);
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

			//TODO::What happens if topic not found or could not subscribe
			manager_->subscribe(topic_name);
		
		}

		bool Subscribe::publish()
		{
			  			std::cout <<"Subscribe::publish()....map sizea";
			  			std::cout <<topic_subscirbtions_.size()<<std::endl;
			for (std::map<std::string,std::shared_ptr<Subscribtion>>::iterator it= topic_subscirbtions_.begin()
					;it!=topic_subscirbtions_.end(); ++it ){
				if (it->second->publish())//is it time to publish
					manager_->publish(it->second->topic_);
			}

			return true;
		
		}


//------------Subscribtion Object


		Subscribtion::Subscribtion(fawkes::Clock *clock, std::string client_id, std::string topic)
		:topic_(topic)
		,client_id_(client_id)
		{
			clock_=clock;
		}

		Subscribtion::~Subscribtion(){}

		void
		Subscribtion::subscribe(details *subscirbe_args){
  			
  			std::cout <<"REACHED THE SUBSCRIBTION STAGE WITH "<<topic_<<std::endl;
			details_list_.push_back(subscirbe_args);
			
			last_published_time_=new Time(clock_);
			last_published_time_->stamp();
			std::cout << last_published_time_->in_msec()<<std::endl;	

		}
		
		//Will be called by the publich method in Subscribe  
		//for each iteration to know whither i should publish this topic or not yet

		bool //Return wither its time to publish this topic or not
		Subscribtion::publish(){
			double throttle_rate= details_list_.front()->throttle_rate; //TODO.Choose the smalest throttle rate not just take the first one
			fawkes::Time now(clock_);
			std::cout << last_published_time_->in_msec()<<std::endl;

			if ((now.in_msec() - (*last_published_time_).in_msec()) >= throttle_rate);
			{
				last_published_time_->stamp();
				std::cout << "STAMPED"<<std::endl;	
				return true;
			}
			return false;
		}


		const char* Subscribtion::get_topic_name() {return topic_.c_str();}