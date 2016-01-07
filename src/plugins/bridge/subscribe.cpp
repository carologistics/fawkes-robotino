#include "subscribe.h"

#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include <cstdio>

#include <utils/time/time.h>
#include <core/threading/mutex.h>



using namespace fawkes;

		bool compare_throttle_rate(details *first, details *second){return (first->throttle_rate <= second->throttle_rate);	}

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
					//if (manager_->subscribe(topic_name))
						topic_subscirbtions_[topic_name]= std::make_shared<Subscribtion>(clock_,client_id_,topic_name);
					//else
					//	return;
			}


			details *subscribe_args=new details();
			if(d.HasMember("id"))
				subscribe_args->subscribtion_id=std::string(d["id"].GetString());
			if(d.HasMember("type"))
				subscribe_args->msg_type=std::string(d["type"].GetString());
			if(d.HasMember("throttle_rate"))
				subscribe_args->throttle_rate=d["throttle_rate"].GetInt();
			if(d.HasMember("queue_length"))
				subscribe_args->queue_length=d["queue_length"].GetInt();
			if(d.HasMember("fragment_size"))
				subscribe_args->fragment_size=d["fragment_size"].GetInt();
			if(d.HasMember("compression"))
				subscribe_args->compression=std::string(d["compression"].GetString());	

			topic_subscirbtions_[topic_name]->subscribe(subscribe_args);

		
		}

		bool Subscribe::publish()
		{
			// std::cout <<"Subscribe::publish()....map sizea";
			// std::cout <<topic_subscirbtions_.size()<<std::endl;

			for (std::map<std::string,std::shared_ptr<Subscribtion>>::iterator it= topic_subscirbtions_.begin()
					;it!=topic_subscirbtions_.end(); ++it ){
				if (it->second->publish())//is it time to publish
					manager_->publish(it->second->topic_);
			}

			return true;
		
		}


		//--------------------------------------------SUBSCBTION


		Subscribtion::Subscribtion(fawkes::Clock *clock, std::string client_id, std::string topic)
		:topic_(topic)
		,client_id_(client_id)
		{
			clock_=clock;
			time_mutex_=new Mutex();
			sub_list_mutex_= new Mutex();
		}

		Subscribtion::~Subscribtion(){}

		void
		Subscribtion::subscribe(details *subscirbe_args){
  			
  			std::cout <<"REACHED THE SUBSCRIBTION STAGE WITH "<<topic_<<std::endl;

  			sub_list_mutex_->lock();
			details_list_.push_back(subscirbe_args);

			details_list_.sort(compare_throttle_rate);
			sub_list_mutex_->unlock();
			
			time_mutex_->lock();
			last_published_time_=new Time(clock_);
			last_published_time_->stamp();
			std::cout << last_published_time_->in_msec()<<std::endl;	
			time_mutex_->unlock();
		}
		
		//Will be called by the publich method in Subscribe  
		//for each iteration 	to know whither i should publish this topic or not yet

		bool //Return wither its time to publish this topic or not
		Subscribtion::publish(){

			sub_list_mutex_->lock();
			int throttle_rate=details_list_.front()->throttle_rate; //should be garantead to be the smallest
			sub_list_mutex_->unlock();

			fawkes::Time now(clock_);
			std::cout << last_published_time_->in_msec()<<std::endl;
			std::cout << throttle_rate <<std::endl;
			std::cout<< (now.in_msec() - (*last_published_time_).in_msec()) <<std::endl;

			if ( (now.in_msec() - (*last_published_time_).in_msec()) >= throttle_rate)
			{
				time_mutex_->lock();
				last_published_time_->stamp();
				time_mutex_->unlock();

				std::cout << "STAMPED"<<std::endl;	
				return true;
			}

			return false;
		}



		const char* Subscribtion::get_topic_name() {return topic_.c_str();}
	