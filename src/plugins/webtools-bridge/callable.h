
#ifndef _CALLABLE_H
#define _CALLABLE_H

#include "event_type.h"
#include <memory>

//class WebSession;
class EventEmitter;

class Callable
{
	public:

		virtual void  callback	( EventType event_type , std::shared_ptr <EventEmitter> handler) ; 
  
};

#endif