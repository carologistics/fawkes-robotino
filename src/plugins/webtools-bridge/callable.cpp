#include "callable.h"
#include "event_emitter.h"


void
Callable::callback( EventType event_type ,std::shared_ptr <EventEmitter> handler)
{
	//the event_type with dynamic_casting for the event_emitter should be enough to uniquily tell you what you want to do
}

// void 
// Callable::register_for_event(EventType event_type , std::shared_ptr <EventEmitter> handler )
// {
//  	handler->register_callback( event_type , shared_from_this() );
// };
 
// void
// Callable::unregister_from_event( EventType event_type ,std::shared_ptr <EventEmitter> handler)
// {
// 	handler->unregister_callback( event_type , shared_from_this() );
// };
