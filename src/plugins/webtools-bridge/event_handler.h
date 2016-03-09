
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

		EventHandler();
		virtual ~EventHandler();//To make it polymorfic (enabling casting from Derived to Base )

		//implement if u want something to happen after emmittion of events
		virtual void do_on_event(EventType event_type) = 0;//ensures (enabling casting from Derived to Base)
		
		//my be name it emitte_event later 
		void call_callbacks(EventType event_type); 

		void register_callback  ( EventType event_type , std::shared_ptr <Callable> callable );
  		void unregister_callback( EventType event_type , std::shared_ptr <Callable> callable );

  	private:
  		std::map<EventType,std::list<std::shared_ptr <Callable> >>					events_to_callable_map_;
    	std::map<EventType,std::list<std::shared_ptr <Callable> >>::iterator		it_events_;
    	std::list<std::shared_ptr <Callable> >::iterator							it_callables_;
};

#endif