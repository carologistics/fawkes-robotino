#include <map>
#include <list>
#include <memory>

#include <rapidjson/document.h>//To be removed from here after serialzer is moved
#include <rapidjson/error/en.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exceptions/software.h>
#include <utils/time/time.h>


#include "subscription_capability.h" 
#include "web_session.h"

using namespace rapidjson;
using namespace fawkes;

//=================================   Subscription  ===================================

Subscription::Subscription(std::string topic_name , std::string prefix, fawkes::Clock * clock)
	:	active_status_(DORMANT)
	, 	topic_name_(topic_name)
	,	processor_prefix_(prefix)
	,	clock_(clock)
	,	finalized (false)
{
	mutex_=new fawkes::Mutex();
}


Subscription::~Subscription()
{
	//delete it from the class created the subscriptiuon (ie. processor)
	//delete clock_;
	delete mutex_;
}

//---------------------INSTACE OPERATIONS
void
Subscription::finalize()
{
	MutexLocker ml(mutex_);
	if(!finalized)
	{
		//If still active deactivate 
		if(is_active()) { deactivate(); }
		//call the extended version
		finalize_impl();

		//Deregister from sessions and remove them
		if(!empty()){

			for(it_subscriptions_ = subscriptions_.begin()
				;it_subscriptions_ != subscriptions_.end()
				;)
			{
				remove_session(it_subscriptions_->first);
				subscriptions_.erase(it_subscriptions_++);
			}
		}

		finalized=true;
	}
}

void
Subscription::activate()
{
	if(!is_active()){
		activate_impl();
		active_status_ = ACTIVE;
	}
}

void
Subscription::deactivate()
{
	if(is_active()){
		deactivate_impl();
		active_status_ = DORMANT;
	}
}

bool
Subscription::is_active()
{
	return (active_status_ == ACTIVE );
}

bool
Subscription::empty()
{
	//This assumes that the clients removale and the removale of their subscribtions were done correctly
	return subscriptions_.empty();
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

/**Subsumes a DORMANT Subscription instace into an ACTIVE one.
 * This is usually called when there is more than one Subscription instance for the same topic.
 * The owning instance must be Active and the instance to be subsumed must to be Dormant
 * ie, ActiveInstance.Subsume(DormantInstance). After the call, the dormant instance could be safly deleted.
 * @param  The Dormant Subscription Instance to subsume
 */
void
Subscription::subsume(std::shared_ptr <Subscription> dormant_subscription)
{

	if (topic_name_ != dormant_subscription->get_topic_name()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}

	if (!is_active()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}

	if (dormant_subscription->is_active()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}


	for(std::map <std::shared_ptr<WebSession> , std::list<Request>>::iterator 
		it_subscriptions = dormant_subscription->subscriptions_.begin()
		;it_subscriptions != dormant_subscription->subscriptions_.end()
		;it_subscriptions ++){

		for(std::list< Request >::iterator 
			it_requests = it_subscriptions ->second.begin() 
			;it_requests != it_subscriptions ->second.end()
			;it_requests ++ ){

			add_request( it_requests ->id
									, it_requests ->compression
									, it_requests ->throttle_rate
									, it_requests ->queue_length
									, it_requests ->fragment_size
									, it_subscriptions ->first);

			//dormant_subscription->remove_request( it_requests ->id , it_subscriptions ->first);

		}
	}
		dormant_subscription->finalize();
}

//---------------------REQUEST HANDLING

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
	MutexLocker ml(mutex_);

	it_subscriptions_ = subscriptions_.find(session);

	//if it is a new session, register my terminate_handler for the session's callbacks
	if(it_subscriptions_ == subscriptions_.end()){
		add_new_session(session);
	}

	//if there was older requests for this session,  point to the same last_published_time
	if ( it_subscriptions_ != subscriptions_.end() && !(subscriptions_[session].empty()) ){
	
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
	MutexLocker ml(mutex_);

	it_subscriptions_ = subscriptions_.find(session);	

	if(it_subscriptions_ == subscriptions_.end()){
		//there is no such session. Maybe session was closed before the request is processed
		return;
	}

	for( it_requests_  = subscriptions_[session].begin()
		;it_requests_  != subscriptions_[session].end()
		;it_requests_ ++){
		
		if((*it_requests_).id == subscription_id){
			subscriptions_[session].erase(it_requests_ );
  			break;
		}
	}

	//sub_list_mutex_->lock();
	if(subscriptions_[session].empty()){
		remove_session(session);
		subscriptions_.erase(session);
	}
	else{
		//Always have the least throttel rate on the top
		subscriptions_[session].sort(compare_throttle_rate);
	}
}

void
Subscription::emitt_event(EventType event_type)
{
	for(it_callables_  = callbacks_ [event_type].begin();
		it_callables_ != callbacks_ [event_type].end() ; 
		it_callables_++)
	{
		(*it_callables_)->callback(event_type , shared_from_this());
	}
}

void 
Subscription::callback(EventType event_type , std::shared_ptr<EventEmitter> event_emitter)
{
	//sub_list_mutex_->lock();
	MutexLocker ml(mutex_);

	try{
		//check if the event emitter was a session
		std::shared_ptr <WebSession> session;
		session = std::dynamic_pointer_cast<WebSession> (event_emitter);
		if(session != NULL)
		{
			if(event_type == EventType::TERMINATE )
			{
				//make sure the session is still there and was not deleted while waiting for the mutex
				if (subscriptions_.find(session) != subscriptions_.end())
					subscriptions_.erase(session);

				std::cout<< "Session terminated NICELY :D" << std::endl;

				//was it the last session? if yes, Subscription emit TERMINATTION event and destories itself.
				if(subscriptions_.empty()){
					std::shared_ptr<Subscription> my_self= shared_from_this();// Just to keep object alive till after its deleted from manager
					emitt_event(EventType::TERMINATE);
					
					ml.unlock();
					my_self->finalize();
					std::cout<< "Subscripton topic terminated!" << std::endl;

					//finalize will need the mutex
				}
				//TODO:check if subscription became empty and trigger the delete from the owning class if that was the case

			}
		}
		
	}
	catch(Exception &e){
		//if exception was fired it only means that the casting failed becasue the emitter is not a session
	}

}

void 
Subscription::add_new_session(std::shared_ptr<WebSession> session)
{
	//register termination handler
	session->register_callback(EventType::TERMINATE , shared_from_this() );
//	subscriptions_[session];// Check if okay
}

void 
Subscription::remove_session(std::shared_ptr<WebSession> session)
{
	//deregister termination handler
	session->unregister_callback(EventType::TERMINATE , shared_from_this() );
//	subscriptions_.erase(session);
}


//--------------Capability Handling

/**This will be called by the whatever event that trigger the publish,Like a periodic clock 
 * or an data update listener).
 * It checks for each session if enough time has passed since its last publish 
 * If so, it serialize the data and send the json. Otherwise  (not sure yet, but 
 * for now just drop it new data. Later maybe queue it depending on the queue size).
 */
void
Subscription::publish()
{
	MutexLocker ml(mutex_);

	if( is_active() )
	{
		std::string prefiexed_topic_name=topic_name_;
	
		for( it_subscriptions_ = subscriptions_.begin() 
			; it_subscriptions_ != subscriptions_.end()
			; it_subscriptions_++)
		{
			if( it_subscriptions_->second.empty() ) 
			{
				//This means unsubscribe() didnt work properly to delete that session after unsubscribing all clients components
				//throw some exception
				remove_session( it_subscriptions_->first);
				subscriptions_.erase(it_subscriptions_++);
				continue;
			}
	
			//Checking if it is time to publish topic
			//sub_list_mutex_->lock();
			fawkes::Time now(clock_);
			//smallest throttle_rate always on the top
			unsigned int throttle_rate = it_subscriptions_->second.front().throttle_rate;
			std::string  id= it_subscriptions_->second.front().id;// could be done here to minimize locking
			unsigned int last_published = it_subscriptions_->second.front().last_published_time->in_msec();
			unsigned int time_passed = (now.in_msec() -last_published ); 
			//sub_list_mutex_->unlock();
	
			if (time_passed >= throttle_rate) {
	
				std::string complete_json_msg = serialize("publish"
												, prefiexed_topic_name
												, id);
	
				//Only stamp if it was sent
				//time_mutex_->lock();
				it_subscriptions_->second.front().last_published_time->stamp();
				//time_mutex_->unlock();

				//Unnecessary right now. No Mutex at session
				//// To avoid deadlock if the session mutex was locked to process a request 
				////(and the request cant add coz ur locking the subscription)
				////ml.unlock();

				//send msg it to session
				it_subscriptions_->first->send(complete_json_msg);
				////ml.relock();
			}
		}
	}
}

//Serialization could be moved later to a Protocol hadleing util
std::string
Subscription::serialize(std::string op
						, std::string prefiexed_topic_name
						, std::string id)
{
	//MutexLocker ml(mutex_);

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

//-------------To be removed

//TODO:replace by a lambda function
bool
Subscription::compare_throttle_rate(Request first, Request second)
{
	return (first.throttle_rate <= second.throttle_rate);
}
