/***************************************************************************
 * event_emitter.cpp - Interface to allow calling of registered callables on registered events
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



#include "callable.h"
#include "event_emitter.h"


#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

EventEmitter::EventEmitter()
{
	mutex_ = new fawkes::Mutex();

}

EventEmitter::~EventEmitter()
{
	callbacks_.clear();
	delete mutex_;
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
	MutexLocker ml(mutex_);

	callbacks_ [event_type].push_back(callable);
}

void
EventEmitter::unregister_callback( EventType event_type , std::shared_ptr <Callable> callable )
{
	MutexLocker ml(mutex_);

	it_callables_ = std::find_if(callbacks_ [event_type].begin(), callbacks_ [event_type].end(), 
		[&](std::shared_ptr<Callable> const& c){return c == callable;});

	if( it_callables_ != callbacks_ [event_type].end() ) {
		callbacks_ [event_type].erase(it_callables_);
	}
}
