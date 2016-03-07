#include "callable.h"
#include "event_handler.h"

void
EventHandler::call_callbacks(EventType event_type)
{
	for(it_callables_  = events_to_callable_map_ [event_type].begin();
		it_callables_ != events_to_callable_map_ [event_type].end() ; 
		it_callables_++)
	{
		(*it_callables_)->callback(event_type , shared_from_this());
	}
	//Clear the whole map to be ready for termination.
	//TODO::move to Finalize() and call it
	//events_to_callable_map_ .clear();
}

void 
EventHandler::register_callback  ( EventType event_type , std::shared_ptr <Callable> callable )
{
	events_to_callable_map_ [event_type].push_back(callable);
}

void
EventHandler::unregister_callback( EventType event_type , std::shared_ptr <Callable> callable )
{
	it_callables_ = std::find_if(events_to_callable_map_ [event_type].begin(), events_to_callable_map_ [event_type].end(), 
		[&](std::shared_ptr<Callable> const& c){return c == callable;});

	if( it_callables_ != events_to_callable_map_ [event_type].end() ) {
		events_to_callable_map_ [event_type].erase(it_callables_);
	}
}
