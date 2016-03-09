#include "callable.h"
#include "event_emitter.h"

EventEmitter::EventEmitter()
{

}

EventEmitter::~EventEmitter()
{
	callbacks_.clear();
}

void
EventEmitter::emitt_event(EventType event_type)
{

	// for(it_callables_  = callbacks_ [event_type].begin();
	// 	it_callables_ != callbacks_ [event_type].end() ; 
	// 	it_callables_++)
	// {
	// 	(*it_callables_)->callback(event_type , shared_from_this());
	// }

	//do_on_event(event_type);
}


void 
EventEmitter::register_callback  ( EventType event_type , std::shared_ptr <Callable> callable )
{
	callbacks_ [event_type].push_back(callable);
}

void
EventEmitter::unregister_callback( EventType event_type , std::shared_ptr <Callable> callable )
{
	it_callables_ = std::find_if(callbacks_ [event_type].begin(), callbacks_ [event_type].end(), 
		[&](std::shared_ptr<Callable> const& c){return c == callable;});

	if( it_callables_ != callbacks_ [event_type].end() ) {
		callbacks_ [event_type].erase(it_callables_);
	}
}
