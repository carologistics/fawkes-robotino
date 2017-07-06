/***************************************************************************
 * event_emitter.h - Interface to allow calling of registered callables on registered events
 *
 *  Created: Mon Mar 2016
 *  Copyright  2016  MosafaGomaa
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

#ifndef _EVENT_HANDLER_H
#define _EVENT_HANDLER_H

#include <memory>
#include <map>
#include <list>
#include <algorithm>

#include "event_type.h"

class Callable;

namespace fawkes{
	class Mutex;
}

class EventEmitter
{
	public:

		EventEmitter();
		virtual ~EventEmitter();//To make it polymorfic (enabling casting from Derived to Base )

		//my be name it emitte_event later 
		virtual void emitt_event(EventType event_type) = 0; 

		void register_callback  ( EventType event_type , std::shared_ptr <Callable> callable );
  		void unregister_callback( EventType event_type , std::shared_ptr <Callable> callable );

  	protected:
  		std::map<EventType,std::list<std::shared_ptr <Callable> >>					callbacks_;
    	std::map<EventType,std::list<std::shared_ptr <Callable> >>::iterator		it_events_;
    	std::list<std::shared_ptr <Callable> >::iterator							it_callables_;
    	fawkes::Mutex *mutex_;
};

#endif