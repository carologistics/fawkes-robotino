/***************************************************************************
 *  bridge_manager.h - Single access point for sessions to the bridge's
 *capabilities. Created: 2016 Copyright  2016 Mostafa Gomaa
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

#ifndef _BRIDGE_MANAGER_H
#define _BRIDGE_MANAGER_H

#include "bridge_processor.h"
#include "capability_manager.h"
#include "web_session.h"

#include <rapidjson/document.h>
#include <rapidjson/error/en.h> //TODO:: move to the cpp

#include <memory>
#include <string>

/*TODO:Change the name of this class suggestions
 * CapabiltiesDispatcher
 * BridgeMother
 * Bridge
 */

class BridgeManager
//:  public std::enable_shared_from_this<BridgeManager>
{
public:
	BridgeManager();
	~BridgeManager();

	void finalize();

	void incoming(std::string JsonMsg, std::shared_ptr<WebSession> session);

	bool deserialize(std::string jsonStr, rapidjson::Document &d);

	bool register_operation_handler(std::string                        operation_name,
	                                std::shared_ptr<CapabilityManager> cpm);

	bool register_processor(std::shared_ptr<BridgeProcessor> processor);

	// bool unregister_operation_handler(std::string operation_name);

	// bool unregister_processor(std::shared_ptr <BridgeProcessor> processor);

private:
	std::map<std::string, std::shared_ptr<CapabilityManager>> operation_cpm_map_;
	// std::map <std::string, WebSessison>
	// clients_;

	/*TO_BE_REFACTOR:register Cpability managers (not operations) and pull the
   provided operations list from the cpm  itself
   Each cpm should store what operations it provides*/
};

#endif