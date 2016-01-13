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
			random_mutex_=new Mutex();

		}

		Subscribe::~Subscribe(){}
		
		void 		
		Subscribe::handle_message(Document &d){
			std::string op_name= std::string(d["op"].GetString());

			if (op_name=="subscribe")
				subscirbe(d);
			else if (op_name=="unsubscribe")
				unsubscribe(d);			
			else
				std::cout<< "No operation with this name in this capability"<<std::endl;
		}

		void 
		Subscribe::subscirbe(Document &d){

			std::cout<< "Subscribing";

			std::string topic_name= std::string(d["topic"].GetString());

			std::cout<< " TO" << topic_name << std::endl;

			if(topic_subscirbtions_.find(topic_name)==topic_subscirbtions_.end()){
					if (manager_->subscribe(topic_name))
						topic_subscirbtions_[topic_name]= std::make_shared<Subscribtion>(clock_,client_id_,topic_name);
					else
						return;
			}

		
			details* subscribe_args=new details();
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

		//return true is there topic no longer exists in the list..Including if it wasnot there originaly
		bool Subscribe::unsubscribe(Document &d){
			std::string topic_name=std::string(d["topic"].GetString());
			if(topic_subscirbtions_.find(topic_name)==topic_subscirbtions_.end()){
				std::cout<<"No topic found to unsubscribe from"<<std::endl;
				//todo::maybe u have to try remove it from manager just in case it was not removed
				return true;
			}
				
			if(d.HasMember("id")){
				std::string subs_id = std::string(d["id"].GetString());
				topic_subscirbtions_[topic_name]->unsubscribe(subs_id);
				
				//was it the last subscribtion
				if(topic_subscirbtions_[topic_name]->get_subscribtion_size() == 0)
				{
					//remove the whole object
					topic_subscirbtions_.erase(topic_name);
					//manager_->unsubscribe(topic_name);
				}
						
				return true;
			}
			else 
				std::cout << "there was no Id send" <<std::endl;

			return false;
		}

		bool Subscribe::publish()
		{
			// std::cout <<"Subscribe::publish()....map sizea";
			// std::cout <<topic_subscirbtions_.size()<<std::endl;

			for (std::map<std::string,std::shared_ptr<Subscribtion>>::iterator it= topic_subscirbtions_.begin()
					;it!=topic_subscirbtions_.end(); ++it ){
				
				if ( it->second->publish())
					manager_->publish(it->second->get_topic_name());
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
			time_mutex_->unlock();
		}
		
		bool
		Subscribtion::unsubscribe(std::string subs_id){
			bool found =false;

			std::cout <<"unsubscribeing from"<<topic_<< "id"<< subs_id<<std::endl;


			for (std::list<details*>::iterator it= details_list_.begin() ;it!=details_list_.end(); ++it ){
				if ( (*it)->subscribtion_id == subs_id && !subs_id.empty() )
				{
					found = true;
  					sub_list_mutex_->lock();
					details_list_.erase(it);
  					sub_list_mutex_->unlock();

					break;
				}
			}	

			return found;		
		}

		bool //Return wither its time to publish this topic or not
		Subscribtion::publish(){

			if(get_subscribtion_size() > 0){

				sub_list_mutex_->lock();
				int throttle_rate=details_list_.front()->throttle_rate; //should be garantead to be the smallest
				std::cout << "id:" <<details_list_.front()->subscribtion_id <<std::endl;	
				std::cout << "throttle_rate rate:" <<throttle_rate <<std::endl;	
				sub_list_mutex_->unlock();


				time_mutex_->lock();
				fawkes::Time now(clock_);
				int time_passed= (now.in_msec() - (*last_published_time_).in_msec()); 
				time_mutex_->unlock();

				if (time_passed >= throttle_rate)
				{
					time_mutex_->lock();
					last_published_time_->stamp();
					time_mutex_->unlock();

					std::cout << "STAMPED"<<std::endl;	
					return true;
				}
			}

			return false;
		}



		std::string Subscribtion::get_topic_name() {return topic_;}

		int Subscribtion::get_subscribtion_size(){
			sub_list_mutex_->lock();
			int num_of_subscribtions = details_list_.size();
			sub_list_mutex_->unlock();

			return num_of_subscribtions;
		}
	