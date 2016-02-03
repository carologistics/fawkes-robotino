#include <map>
#include <list>
#include <memory>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>


#include "subscription_capability.h" 
#include "web_session.h"

using namespace rapidjson;

//=================================   Subscription  ===================================

Subscription::Subscription(std::string topic_name , std::string prefix, fawkes::Clock * clock)
	:	active_status_(DORMANT)
	, 	topic_name_(topic_name)
	,	processor_prefix_(prefix)
	,	clock_(clock)
	,	finalized (false)
{
}


Subscription::~Subscription()
{
	//delete it from the class created the subscriptiuon (ie. processor)
	//delete clock_;
}

void
Subscription::finalize()
{
	if(!finalized)
	{
		if(is_active()) { deactivate(); }
		Subscription::finalize_impl();
		finalized=true;
	}
}

void
Subscription::activate()
{
	if(is_active()){
		activate_impl();
		active_status_ = ACTIVE;
	}
}

void
Subscription::deactivate()
{
	if(!(is_active())){
		deactivate_impl();
		active_status_ = DORMANT;
	}
}

void
Subscription::subsume(std::shared_ptr <Subscription> subscription_to_append)
{
	if (topic_name_ != subscription_to_append->get_topic_name()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}

	for(std::map <std::shared_ptr<WebSession>, std::list<Request> >::iterator 
		 it_subscribers = subscription_to_append->subscriptions_.begin()
		;it_subscribers != subscription_to_append->subscriptions_.end()
		;it_subscribers++){

		for(std::list<Request>::iterator 
			 it_requests = it_subscribers->second.begin() 
			;it_requests != it_subscribers->second.end()
			;it_requests++ ){

			add_request( it_requests->id
									, it_requests->compression
									, it_requests->throttle_rate
									, it_requests->queue_length
									, it_requests->fragment_size
									, it_subscribers->first);
			subscription_to_append->finalize();
		}
	}

}

bool
Subscription::empty()
{
	//This assumes that the clients removale and the removale of their subscribtions were done correctly
	return subscriptions_.empty();
}

bool
Subscription::is_active()
{
	//This assumes that the clients removale and the removale of their subscribtions were done correctly
	return (active_status_ == ACTIVE );
}

std::string
Subscription::get_topic_name()
{
	return topic_name_;
}

std::string
Subscription::get_processor_prefix()
{
	return processor_prefix_;
}

/*This will be called by the whatever event that trigger the publish with the json msg.
*(Like a periodic clock or an data update listener).
*It checks for each session if enough time has passed since the last publish 
*(ie, > throttle_rate). If so, it sends the json. Otherwise  (not sure yet, but 
*for now just drop it. Later maybe queue it).
*/
void
Subscription::publish()
{
	if( is_active() )
	{
		std::string prefiexed_topic_name= "/"+processor_prefix_+"/"+topic_name_;
	
		for(std::map <std::shared_ptr<WebSession> , std::list<Request>>::iterator 
			it = subscriptions_.begin() 
			; it != subscriptions_.end()
			; it++)
		{
			if( it->second.empty() ) 
			{
				//This means unsubscribe() didnt work properly to delete that session after unsubscribing all clients components
				//throw some exception
				subscriptions_.erase(it);
				continue;
			}
	
			//Checking if it is time to publish topic
			//sub_list_mutex_->lock();
			fawkes::Time now(clock_);
			//smallest throttle_rate always on the top
			unsigned int throttle_rate = it->second.front().throttle_rate;
			std::string  id= it->second.front().id;// could be done here to minimize locking
			unsigned int last_published = it->second.front().last_published_time->in_msec();
			unsigned int time_passed = (now.in_msec() -last_published ); 
			//sub_list_mutex_->unlock();
	
			if (time_passed >= throttle_rate) {
	
				std::string complete_json_msg = serialize("publish"
												, prefiexed_topic_name
												, id);
				//send msg it to session
				it->first->send(complete_json_msg);
				//catch exception
	
				//Only stamp if it was sent
				//time_mutex_->lock();
				it->second.front().last_published_time->stamp();
				//time_mutex_->unlock();
			}
		}
	}
}

/*this should be called by each subscribe() call to add the request and the requesting session*/
void
Subscription::add_request( std::string id 		
						, std::string compression
						, unsigned int throttle_rate	
						, unsigned int queue_length 	
						, unsigned int fragment_size 	
						, std::shared_ptr<WebSession> session)
{
	Request request;
	//CHANGE:this matches for the pointer not the object

	std::map < std::shared_ptr<WebSession> , std::list<Request>>::iterator it;

	it = std::find_if(subscriptions_.begin(), subscriptions_.end() 
					,	[session](const std::pair< std::shared_ptr<WebSession> , std::list<Request>> & t) 
						-> bool 
						{ 
			     			 return t.first->get_id() == session->get_id();
			  			} 
			  		);

	//if it is a new session, register my terminate_handler for the session's callbacks
	if(it == subscriptions_.end()){
		register_session_handlers(session);
	}

	//if there was older requests for this session,  point to the same last_published_time
	if ( it != subscriptions_.end() && !(subscriptions_[session].empty()) ){
	
		// if(subscriptions_[session].find(id) != subscriptions_[session].end())
		// {
		// 	//throw exception..That id already exists for that client on that topic
		// }
			
		request.last_published_time = subscriptions_[session].front().last_published_time;
	}else{
		request.last_published_time = std::make_shared<fawkes::Time> (clock_);
	}
	
	request.id=id;
	request.compression=compression;
	request.throttle_rate=throttle_rate;
	request.queue_length=queue_length;
	request.fragment_size=fragment_size;

	//sub_list_mutex_->lock();
	subscriptions_[session].push_back(request);
	subscriptions_[session].sort(compare_throttle_rate);//replace by a lambda function
	//sub_list_mutex_->unlock();
}

/*
this should be called by each unsubscribe() to remove the request and posibly the requesting session
*/
void
Subscription::remove_request(std::string subscription_id, std::shared_ptr <WebSession> session)
{
	//TODO:: lock by mutex
	std::map <std::shared_ptr<WebSession> , std::list<Request>>::iterator it;

	it = std::find_if(subscriptions_.begin(), subscriptions_.end() 
					,	[session](const std::pair<std::shared_ptr<WebSession> , std::list<Request> > & t) 
						-> bool 
						{ 
			     			 return t.first->get_id() == session->get_id();
			  			} 
			  		);	
	//TODO:: make sure there is only one session object per session. Otherwise implement an equality operator
	if(it != subscriptions_.end()){
		//there is no such session. Maybe session was closed before the request is processed
		return;
	}

	for( std::list< Request >::iterator 
		 it = subscriptions_[session].begin()
		;it != subscriptions_[session].end()
		;it++){
		
		if((*it).id == subscription_id){
			//sub_list_mutex_->lock();
			subscriptions_[session].erase(it);
  			//sub_list_mutex_->unlock();
  			break;
		}
	}

	//sub_list_mutex_->lock();
	if(subscriptions_[session].empty()){
		subscriptions_.erase(session);
	}
	//sub_list_mutex_->lock();
}

void 
Subscription::terminate_session_handler(std::shared_ptr<WebSession> session)
{
	//sub_list_mutex_->lock();
	std::cout<< "Session terminated NICELY :D" << std::endl;
	subscriptions_.erase(session);
	//sub_list_mutex_->lock();
	//check if subscription became empty and trigger the delete from the owning class if that was the case
}

void 
Subscription::register_session_handlers(std::shared_ptr<WebSession> session)
{
	session->register_terminate_callback( boost::bind(&Subscription::terminate_session_handler,this,_1) );
}

void
Subscription::finalize_impl()
{
	//Override to extend behavior
}

void
Subscription::activate_impl()
{
	//Override to extend behavior
}

void
Subscription::deactivate_impl()
{
	//Override to extend behavior
}

//Serialization could be moved later to a Protocol hadleing util
std::string
Subscription::serialize(std::string op
						, std::string prefiexed_topic_name
						, std::string id)
{

	StringBuffer s;
  	Writer<StringBuffer> writer(s);      
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(),(SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(),(SizeType)id.length());
	
	writer.String("topic");
	writer.String(prefiexed_topic_name.c_str(), (SizeType)prefiexed_topic_name.length());
	
	writer.String("msg");
	writer.StartObject();
	//Serialize you data json here
	writer.EndObject();	//End if data json

	writer.EndObject();//End of complete Json_msg 
    
    //std::cout << s.GetString() << std::endl;

    return s.GetString();
}

//TODO:replace by a lambda function
bool
Subscription::compare_throttle_rate(Request first, Request second)
{
	return (first.throttle_rate <= second.throttle_rate);
}
