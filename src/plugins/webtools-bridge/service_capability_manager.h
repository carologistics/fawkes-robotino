/***************************************************************************
 *  service_capability_manager.h - Service Requests Dispatcher
 *  Created: 2016
 *  Copyright  2016 Mostafa Gomaa
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

#include "capability_manager.h"

class ServiceCapability;

class ServiceCapabilityManager : public CapabilityManager
{
public:
	ServiceCapabilityManager();
	~ServiceCapabilityManager();

	void handle_message(rapidjson::Document &d, std::shared_ptr<WebSession> session);

	bool register_processor(std::shared_ptr<BridgeProcessor> processor);
};
