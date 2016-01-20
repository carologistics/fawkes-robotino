
#include <map>
#include <list>
#include <memory>

#include "Subscription_capability.h" 
#include "web_session.h"

using namespace rapidjson;

//=================================   Subscription  ===================================


Subscription::Subscription(std::string topic_name , std::string prefix, fawkes::Clock * clock)
	: 	topic_name_(topic_name)
	,	processor_prefix(prefix)
	,	clock_(clock)
	,	active_status(DORMANT)
{

}


Subscription::~Subscription()
{
	delete clock;
}

void
Subscription::activate()
{
	//implement such that is only active subscribtion for a topic is active at anypoint of time
	//
	active_status_ = ACTIVE;
}

void
Subscription::deactivate()
{
	active_status_ = DORMANT;
}

void
Subscription::subsume(std::shared_ptr <Subscription> subscription_to_append)
{
	if (this.topic_name_ != subscription_to_append.get_topic_name()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}

	for(std::map <std::shared_ptr, RequestList>::iterator 
		 it_subscribers = subscription_to_append.subscribers_.begin()
		;it_subscribers != subscription_to_append.subscribers_.end()
		;it_subscribers++){

		for(RequestList::iterator 
			 it_requests = it_subscribers->second.begin() 
			;it_requests != it_subscribers->second.end()
			;it_requests++ ){

			add_Subscription_request(it_subscribers->first, *it_requests);
		}
	}

}

bool
Subscription::empty()
{
	//This assumes that the clients removale and the removale of their subscribtions were done correctly
	return subscribers_.empty();
}

void
Subscription::finalize()
{
	//close all the processor specefic interfaces or listenners or whatever u used
}






/*
this should be called by each subscribe() call to add the request and the requesting session
*/

void
Subscription::add_Subscription_request(SubscriptionRequest request
								   	, std::shared_ptr<WebSession> session);
{
	
	add_Subscription_request( request.id
							, request.compression
							, request.throttle_rate
							, request.queue_length
							, request.fragment_size
							, session);
}

void
Subscription::add_Subscription_request( std::string id 		
									, std::string compression
									, unsigned int throttle_rate	
									, unsigned int queue_length 	
									, unsigned int fragment_size 	
								   	, std::shared_ptr<WebSession> session);
{
	SubscriptionRequest request;

	if ( subscribers_.find(session) != subscribers_.end() && !(subscribers_[session].empty()) ){
		//if there was old Subscriptions point to the same time_object
		if(subscribers_[session].find(id) != subscribers_[session].end())
		{
			//throw exception..That id already exists for that client on that topic
		}
			
		request.last_published_time = subscribers_[session].front().last_published_time;
	}else{
		request.last_published_time = std::make_shared<fawkes::Time> (clock_);
	}
	
	request.id=id;
	request.compression=compression;
	request.throttle_rate=throttle_rate;
	request.queue_length=queue_length;
	request.fragment_size=fragment_size;

	//sub_list_mutex_->lock();
	subscribers_[session].push_back(request);
	subscribers_[session].sort(compare_throttle_rate);//replace by a lambda function
	//sub_list_mutex_->unlock();
}

/*
this should be called by each unsubscribe() to remove the request and posibly the requesting session
*/
void
Subscription::remove_Subscription_request(std::string subscription_id, std::shared_ptr <WebSession> session)
{
	//TODO:: lock by mutex

	//TODO:: make sure there is only one session object per session. Otherwise implement an equality operator
	if(subscribers_.find(session) != subscribers_.end()){

		//throw Exception that the subscirber does not exist 
	}

	for(RequestList::iterator 
		 it = subscribers_[session].begin()
		;it != subscribers_[session].end()
		;it++){
		
		if((*it).id == Subscription_id){
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
Subscription::publish(std::string json_str)
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
Subscription::compare_throttle_rate(SubscriptionRequest first, SubscriptionRequest second)
{
	return (first.throttle_rate <= second.throttle_rate);
}
