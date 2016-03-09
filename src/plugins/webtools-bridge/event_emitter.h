
#ifndef _EVENT_HANDLER_H
#define _EVENT_HANDLER_H

#include <memory>
#include <map>
#include <list>
#include <algorithm>

#include "event_type.h"

class Callable;

class EventEmitter
{
	public:

		EventEmitter();
		virtual ~EventEmitter();//To make it polymorfic (enabling casting from Derived to Base )

		//my be name it emitte_event later 
		virtual void call_callbacks(EventType event_type) = 0; 

		void register_callback  ( EventType event_type , std::shared_ptr <Callable> callable );
  		void unregister_callback( EventType event_type , std::shared_ptr <Callable> callable );

  	protected:
  		std::map<EventType,std::list<std::shared_ptr <Callable> >>					events_to_callable_map_;
    	std::map<EventType,std::list<std::shared_ptr <Callable> >>::iterator		it_events_;
    	std::list<std::shared_ptr <Callable> >::iterator							it_callables_;
};

#endif