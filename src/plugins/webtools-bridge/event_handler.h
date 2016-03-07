
#ifndef _EVENT_HANDLER_H
#define _EVENT_HANDLER_H

#include <memory>
#include <map>
#include <list>
#include <algorithm>

#include "event_type.h"

class Callable;

class EventHandler
: public std::enable_shared_from_this<EventHandler>
{
	public:
		//TODO::add a finialize and de/constructor

		//my be name it handle_event later
		void call_callbacks(EventType event_type); 

	protected:
		void register_callback  ( EventType event_type , std::shared_ptr <Callable> callable );
  		void unregister_callback( EventType event_type , std::shared_ptr <Callable> callable );

  	private:
  		std::map<EventType,std::list<std::shared_ptr <Callable> >>					events_to_callable_map_;
    	std::map<EventType,std::list<std::shared_ptr <Callable> >>::iterator		it_events_;
    	std::list<std::shared_ptr <Callable> >::iterator						it_callables_;
};

#endif