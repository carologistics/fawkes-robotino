#include <map>
#include <list>
#include <memory>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exceptions/software.h>
#include <utils/time/time.h>

#include "advertisment_capability.h" 
#include "web_session.h"

using namespace fawkes;

//=================================   Advertisment  ===================================

Advertisment::Advertisment(std::string topic_name , std::string prefix)
	:	active_status_(DORMANT)
	, 	topic_name_(topic_name)
	,	processor_prefix_(prefix)
	,	finalized (false)
{
	__mutex=new fawkes::Mutex();
}


Advertisment::~Advertisment()
{
	advertisments_.clear();
	delete __mutex;
}

//---------------------INSTACE OPERATIONS
void
Advertisment::finalize()
{
	MutexLocker ml(__mutex);

	if(!finalized)
	{
		//If still active deactivate 
		if(is_active()) { deactivate(); }
		//call the extended version
		finalize_impl();

		//Deregister from sessions and remove them
		if(!empty()){

			for(it_advertisments_ = advertisments_.begin()
				;it_advertisments_ != advertisments_.end()
				;)
			{
				it_advertisments_->first->unregister_callback(EventType::TERMINATE , shared_from_this()); 
				it_advertisments_->second.clear();
				advertisments_.erase(it_advertisments_++);
			}
		}

		finalized=true;
	}
}

void
Advertisment::activate()
{
	if(!is_active()){
		activate_impl();
		active_status_ = ACTIVE;
	}
}

void
Advertisment::deactivate()
{
	if(is_active()){
		deactivate_impl();
		active_status_ = DORMANT;
	}
}

bool
Advertisment::is_active()
{
	return (active_status_ == ACTIVE );
}

bool
Advertisment::empty()
{
	//This assumes that the clients removale and the removale of their subscribtions were done correctly
	return advertisments_.empty();
}


std::string
Advertisment::get_topic_name()
{
	return topic_name_;
}

std::string
Advertisment::get_processor_prefix()
{
	return processor_prefix_;
}

void
Advertisment::finalize_impl()
{
	//Override to extend behavior
}

void
Advertisment::activate_impl()
{
	//Override to extend behavior
}

void
Advertisment::deactivate_impl()
{
	//Override to extend behavior
}

/**Subsumes a DORMANT Advertisment instace into an ACTIVE one.
 * This is usually called when there is more than one Advertisment instance for the same topic.
 * The owning instance must be Active and the instance to be subsumed must to be Dormant
 * ie, ActiveInstance.Subsume(DormantInstance). After the call, the dormant instance could be safly deleted.
 * @param  The Dormant Advertisment Instance to subsume
 */
void
Advertisment::subsume(std::shared_ptr <Advertisment> dormant_advertisment)
{

	if (topic_name_ != dormant_advertisment->get_topic_name()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}

	if (!is_active()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}

	if (dormant_advertisment->is_active()){
		//throw exceptoin that they dont belong to the same topic and cant be merged
		return;
	}


	for(std::map <std::shared_ptr<WebSession> , std::list<Request>>::iterator 
		it_advertisments = dormant_advertisment->advertisments_.begin()
		;it_advertisments != dormant_advertisment->advertisments_.end()
		;it_advertisments ++){

		for(std::list< Request >::iterator 
			it_requests = it_advertisments ->second.begin() 
			;it_requests != it_advertisments ->second.end()
			;it_requests ++ ){

			add_request( it_requests ->id , it_advertisments ->first);
		}
	}
		dormant_advertisment->finalize();
}

//---------------------REQUEST HANDLING

/*this should be called by each subscribe() call to add the request and the requesting session*/
void
Advertisment::add_request( std::string id , std::shared_ptr<WebSession> session)
{

	Request request;
	//CHANGE:this matches for the pointer not the object
	MutexLocker ml(__mutex);

	it_advertisments_ = advertisments_.find(session);

	//if it is a new session, register my terminate_handler for the session's callbacks
	if(it_advertisments_ == advertisments_.end()){
		session->register_callback(EventType::TERMINATE , shared_from_this() );
	}

	// //if there was older requests for this session,  point to the same last_published_time
	// if ( it_advertisments_ != advertisments_.end() && !(advertisments_[session].empty()) )
	// {
	// 	if(advertisments_[session].find(id) != advertisments_[session].end())
	// 	{
	// 		//throw exception..That id already exists for that client on that topic
	// 	}
	// }
	
	request.id=id;

	advertisments_[session].push_back(request);
}

/*
this should be called by each unadvertive() to remove the request and posibly the requesting session
*/
void
Advertisment::remove_request(std::string advertisment_id, std::shared_ptr <WebSession> session)
{
	MutexLocker ml(__mutex);

	it_advertisments_ = advertisments_.find(session);	

	if(it_advertisments_ == advertisments_.end()){
		//there is no such session. Maybe session was closed before the request is processed
		return;
	}

	for( it_requests_  = advertisments_[session].begin()
		;it_requests_  != advertisments_[session].end()
		;it_requests_ ++){
		
		if((*it_requests_).id == advertisment_id){
			advertisments_[session].erase(it_requests_ );
  			break;
		}
	}

	//sub_list_mutex_->lock();
	if(advertisments_[session].empty()){
		session->unregister_callback(EventType::TERMINATE , shared_from_this() );
		advertisments_.erase(session);
	}
}

void
Advertisment::emitt_event(EventType event_type)
{
	for(it_callables_  = callbacks_ [event_type].begin();
		it_callables_ != callbacks_ [event_type].end() ; 
		it_callables_++)
	{
		(*it_callables_)->callback(event_type , shared_from_this());
	}
}

void 
Advertisment::callback(EventType event_type , std::shared_ptr<EventEmitter> event_emitter)
{
	//sub_list_mutex_->lock();
	MutexLocker ml(__mutex);

	try{
		//check if the event emitter was a session
		std::shared_ptr <WebSession> session;
		session = std::dynamic_pointer_cast<WebSession> (event_emitter);
		if(session != NULL)
		{
			if(event_type == EventType::TERMINATE )
			{
				//make sure the session is still there and was not deleted while waiting for the mutex
				if (advertisments_.find(session) != advertisments_.end())
					advertisments_.erase(session);

				std::cout<< "Session terminated NICELY :D" << std::endl;

				//was it the last session? if yes, Advertisment emit TERMINATTION event to the Advertisment_Manager.
				if(advertisments_.empty()){
					std::shared_ptr<Advertisment> my_self= shared_from_this();// Just to keep object alive till after its deleted from manager
					ml.unlock();
					
					emitt_event(EventType::TERMINATE);
					
					//ml.unlock();
					//my_self->finalize();
					std::cout<< "Advertisment topic terminated!" << std::endl;

					//finalize will need the mutex
				}
				//TODO:check if advertisment became empty and trigger the delete from the owning class if that was the case
			}
		}
		
	}
	catch(Exception &e){
		//if exception was fired it only means that the casting failed becasue the emitter is not a session
	}
}
