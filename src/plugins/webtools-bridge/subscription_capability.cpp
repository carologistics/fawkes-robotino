#include <map>
#include <list>
#include <memory>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"


#include "subscription_capability.h" 
#include "web_session.h"

using namespace rapidjson;

//=================================   Subscription  ===================================


Subscription::Subscription(std::string topic_name , std::string prefix, fawkes::Clock * clock)
	: 	topic_name_(topic_name)
	,	processor_prefix_(prefix)
	,	clock_(clock)
	,	active_status_(DORMANT)
{

}


Subscription::~Subscription()
{
	//delete it from the class created the subscriptiuon (ie. processor)
	//delete clock_;
}

void
Subscription::activate()
{
	activate_impl();
	active_status_ = ACTIVE;
}

void
Subscription::deactivate()
{
	Subscription::deactivate_impl();
	active_status_ = DORMANT;
}

void
Subscription::finalize()
{
	Subscription::finalize_impl();
}

//Serialization could be moved later to a Protocol hadleing util
std::string
Subscription::serialize(std::string op
						, std::string prefiexed_topic_name
						, std::string id)
{
	std::string msg=serialize_impl();//the msg feild of the json_msg

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
	writer.String(msg.c_str(), (SizeType)msg.length());

	writer.EndObject();//the full JSON_msg 
    std::cout << s.GetString() << std::endl;

    return s.GetString();
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

void
Subscription::finalize_impl()
{
	//Override to extend behavior
}


std::string
Subscription::serialize_impl()
{
	//Override to extend behavior
	return "";
}



void
Subscription::subsume(std::shared_ptr <Subscription> subscription_to_append)
{
	if (topic_name_ != subscription_to_append->get_topic_name()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}

	for(std::map <std::shared_ptr<WebSession>, RequestList>::iterator 
		 it_subscribers = subscription_to_append->subscribers_.begin()
		;it_subscribers != subscription_to_append->subscribers_.end()
		;it_subscribers++){

		for(RequestList::iterator 
			 it_requests = it_subscribers->second.begin() 
			;it_requests != it_subscribers->second.end()
			;it_requests++ ){

			add_Subscription_request( it_requests->id
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
	return subscribers_.empty();
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


/*
this should be called by each subscribe() call to add the request and the requesting session
*/

void
Subscription::add_Subscription_request( std::string id 		
									, std::string compression
									, unsigned int throttle_rate	
									, unsigned int queue_length 	
									, unsigned int fragment_size 	
								   	, std::shared_ptr<WebSession> session)
{
	SubscriptionRequest request;
	//CHANGE:this matches for the pointer not the object

	std::map <std::shared_ptr<WebSession> , RequestList>::iterator it;

	it = std::find_if(subscribers_.begin(), subscribers_.end() 
					,	[session](const std::pair<std::shared_ptr<WebSession> , RequestList> & t) 
						-> bool 
						{ 
			     			 return t.first->get_id() == session->get_id();
			  			} 
			  		);	

	//if there was old Subscriptions point to the same time_object
	if (it != subscribers_.end() && !(subscribers_[session].empty()) ){
	
		// if(subscribers_[session].find(id) != subscribers_[session].end())
		// {
		// 	//throw exception..That id already exists for that client on that topic
		// }
			
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

	std::map <std::shared_ptr<WebSession> , RequestList>::iterator it;

	it = std::find_if(subscribers_.begin(), subscribers_.end() 
					,	[session](const std::pair<std::shared_ptr<WebSession> , RequestList> & t) 
						-> bool 
						{ 
			     			 return t.first->get_id() == session->get_id();
			  			} 
			  		);	
	//TODO:: make sure there is only one session object per session. Otherwise implement an equality operator
	if(it != subscribers_.end()){

		//throw Exception that the subscirber does not exist 
	}

	for(RequestList::iterator 
		 it = subscribers_[session].begin()
		;it != subscribers_[session].end()
		;it++){
		
		if((*it).id == subscription_id){
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
Subscription::publish()
{

	std::string prefiexed_topic_name= "/"+processor_prefix_+"/"+topic_name_;

	for(std::map <std::shared_ptr<WebSession> , RequestList>::iterator 
		it = subscribers_.begin() 
		; it != subscribers_.end()
		; it++)
	{
		if( it->second.empty() ) 
		{
			//This means unsubscribe() didnt work properly to delete that session after unsubscribing all clients components
			//throw some exception
			subscribers_.erase(it);
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

//TODO:replace by a lambda function
bool
Subscription::compare_throttle_rate(SubscriptionRequest first, SubscriptionRequest second)
{
	return (first.throttle_rate <= second.throttle_rate);
}
