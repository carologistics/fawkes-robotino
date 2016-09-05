
/***************************************************************************
 * callable.cpp - Interface For The Callbacks To Be Called By The EventEmitter
 *
 *  Created: Mon April 11 2016
 *  Copyright  21016 Mostafa Gomaa 
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */



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
