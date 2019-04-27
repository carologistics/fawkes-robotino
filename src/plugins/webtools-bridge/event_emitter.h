/***************************************************************************
 * event_emitter.h - Interface to allow calling of registered callables on
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

#ifndef _EVENT_HANDLER_H
#define _EVENT_HANDLER_H

#include "event_type.h"

#include <algorithm>
#include <list>
#include <map>
#include <memory>

class Callable;

namespace fawkes {
class Mutex;
}

class EventEmitter
{
public:
	EventEmitter();
	virtual ~EventEmitter(); // To make it polymorfic (enabling casting from
	                         // Derived to Base )

	virtual void emitt_event(EventType event_type) = 0;

	void register_callback(EventType event_type, std::shared_ptr<Callable> callable);
	void unregister_callback(EventType event_type, std::shared_ptr<Callable> callable);

protected:
	/// Registry for each EventType and the a coresponding list of classes
	/// registered for it
	std::map<EventType, std::list<std::shared_ptr<Callable>>> callbacks_;
	/// Iterator for the registry
	std::map<EventType, std::list<std::shared_ptr<Callable>>>::iterator it_events_;
	/// Iterator for list of callables
	std::list<std::shared_ptr<Callable>>::iterator it_callables_;
	/// Mutex to lock the registry during operations
	fawkes::Mutex *mutex_;
};

#endif