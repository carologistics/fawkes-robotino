
#ifndef _CALLABLE_H
#define _CALLABLE_H

#include "event_type.h"
#include <memory>

//class WebSession;
class EventHandler;

class Callable
: public std::enable_shared_from_this<Callable>
{
	public:

		virtual void  callback	( EventType event_type , std::shared_ptr <EventHandler> handler) ; 

	protected:
		 void register_for_event	( EventType event_type , std::shared_ptr <EventHandler> handler );
  		 void unregister_from_event	( EventType event_type , std::shared_ptr <EventHandler> handler );
  	
};

#endif