
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

/** @class Callable "callable.h"
 * Abstract class, allowing parents implementing it to get callbacks when events
 * they registered for are triggered
 *
 * @author Mostafa Gomaa
 */

/** Callback to Handle Events.
 * @param event_type type of the event that caused this callback method to be
 * called.
 * @param handler the object instance initiated the callback method call.
 */
void
Callable::callback(EventType event_type, std::shared_ptr<EventEmitter> handler)
{
	// the event_type with dynamic_casting for the event_emitter should be enough
	// to uniquily tell you what you want to do
}