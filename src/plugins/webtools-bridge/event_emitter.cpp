/***************************************************************************
 * event_emitter.cpp - Interface to allow calling of registered callables on
 *registered events
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

#include "event_emitter.h"

#include "callable.h"

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class EventEmitter "event_emitter.h"
 * Implantation of an Event Emitter.
 * Callable classes can register to be notified for certain expected events.
 * The EventEmitter can emitt_events, calling the callbacks of callable classes
 * registered for this event type.
 *
 * @author Mostafa Gomaa
 */

/** Constructor. */
EventEmitter::EventEmitter()
{
	mutex_ = new fawkes::Mutex();
}

/** Destructor. */
EventEmitter::~EventEmitter()
{
	callbacks_.clear();
	delete mutex_;
}

/** Call the callbacks of the Callable Objects
 * By calling this method with an Event type all callbacks
 * of Callables registered to that event will be called.
 * Used to emulates signal emitting.
 * @param event_type what ever event you would like to signal (ex, TERMINATE)
 */
void
EventEmitter::emitt_event(EventType event_type)
{
	// for(it_callables_  = callbacks_ [event_type].begin();
	// 	it_callables_ != callbacks_ [event_type].end() ;
	// 	it_callables_++)
	// {
	// 	(*it_callables_)->callback(event_type , shared_from_this());
	// }

	// do_on_event(event_type);
}

/** Register a callable to be called on some event type
 * @param event_type to receive a callback for
 * @param callable the class with the callback that handles the event.
 */
void
EventEmitter::register_callback(EventType event_type, std::shared_ptr<Callable> callable)
{
	MutexLocker ml(mutex_);

	callbacks_[event_type].push_back(callable);
}

/** Unregister a callable from emitter entries
 * @param event_type that the callable was registered to
 * @param callable to the class that wants to unregister.
 */
void
EventEmitter::unregister_callback(EventType event_type, std::shared_ptr<Callable> callable)
{
	MutexLocker ml(mutex_);

	it_callables_ = std::find_if(callbacks_[event_type].begin(),
	                             callbacks_[event_type].end(),
	                             [&](std::shared_ptr<Callable> const &c) { return c == callable; });

	if (it_callables_ != callbacks_[event_type].end()) {
		callbacks_[event_type].erase(it_callables_);
	}
}
