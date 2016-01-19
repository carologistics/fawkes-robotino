
#include <map>
#include <list>
#include <memory>

#include "subscribtion_capability.h" 
#include "web_session.h"

using namespace rapidjson;

//=================================   SUBSCRIBTION  ===================================


Subscribtion::Subscribtion(std::string topic_name , std::string prefix, fawkes::Clock * clock)
	: 	topic_name_(topic_name)
	,	processor_prefix(prefix)
	,	clock_(clock)
{
}

/*
this should be called by each subscribe() call to add the request and the requesting session
*/
void
Subscribtion::add_subscribtion_request(Document &d, std::shared_ptr <WebSession> session)
{
	SubscribtionRequest request;

	if(d.HasMember("id")) 			request.id 				= 	std::string(d["id"].GetString());
	if(d.HasMember("compression")) 	request.compression		=	std::string(d["compression"].GetString());	
	if(d.HasMember("throttle_rate"))request.throttle_rate	=	d["throttle_rate"].GetUint();
	if(d.HasMember("queue_length")) request.queue_length 	=	d["queue_length"].GetUint();
	if(d.HasMember("fragment_size"))request.fragment_size 	=	d["fragment_size"].GetUint();

	if (subscribers_.find(session) != subscribers_.end() 
						&& !subscribers_[session].empty()){
		//if there was old subscribtions point to the same time_object
		request.last_published_time = subscribers_[session].front().last_published_time;
	}else{
		request.last_published_time = new fawkes::Time(clock_);
	}

	//sub_list_mutex_->lock();
	subscribers_[session].push_back(request);
	subscribers_[session].sort(compare_throttle_rate);//replace by a lambda function
	//sub_list_mutex_->unlock();
}

/*
this should be called by each unsubscribe() to remove the request and posibly the requesting session
*/
void
Subscribtion::remove_subscribtion_request(Document &d, std::shared_ptr <WebSession> session)
{
	//TODO:: lock by mutex

	if(subscribers_.find(session) != subscribers_.end()){
		//throw Exception that the subscirber does not exist 
	}

	if(!d.HasMember("id")){
		//throw Exception that  there was no id in msg ..Wrong format			
	}
	
	std::string subscribtion_id = std::string(d["id"].GetString());
	
	for(RequestList::iterator it = subscribers_[session].begin()
						;it != subscribers_[session].end(); it++){
		
		if((*it).id == subscribtion_id){
			//sub_list_mutex_->lock();
			subscribers_[session].erase(it);
  			//sub_list_mutex_->unlock();
  			break;
		}
	}

	if(subscribers_[session].empty()){
		//sub_list_mutex_->lock();
		subscribers_.erase(session);
		//sub_list_mutex_->lock();
	}
}

/*This will be called by the whatever event that trigger the publish with the json msg.
*(Like a periodic clock or an data update listener).
*It checks for each session if enough time has passed since the last publish 
*(ie, > throttle_rate). If so, it sends the json. Otherwise  (not sure yet, but 
*for now just drop it. Later maybe queue it).
*/
void
Subscribtion::publish(std::string json_str)
{
	for(std::map <std::shared_ptr<WebSession> , RequestList>::iterator it =
	 subscribers_.begin() ; it != subscribers_.end(); it++)
	{
		if( it->second.empty() ) {
			//This means unsubscribe() didnt work properly to delete that session after unsubscribing all clients components
			//throw some exception
			subscribers_.erase(it);
			continue;
		}

		//sub_list_mutex_->lock();
		fawkes::Time now(clock_);
		//smallest throttle_rate always on the top
		unsigned int throttle_rate = it->second.front().throttle_rate;
		unsigned int last_published = it->second.front().last_published_time->in_msec();
		unsigned int time_passed = (now.in_msec() -last_published ); 
		//sub_list_mutex_->unlock();

		if (time_passed >= throttle_rate) {
			//send it to session
			it->first->send(json_str);
			//catch exception

			//Only stamp if it was sent
			//time_mutex_->lock();
			it->second.front().last_published_time->stamp();
			//time_mutex_->unlock();
		}
	}
}

//TODO:replace by a lambda function
bool
Subscribtion::compare_throttle_rate(SubscribtionRequest first, SubscribtionRequest second)
{
	return (first.throttle_rate <= second.throttle_rate);
}
