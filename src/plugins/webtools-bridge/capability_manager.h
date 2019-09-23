
/***************************************************************************
 * capability_manager.h - Interface For Capability Managers
 *
 *  Created:  2016
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

#ifndef _CAPABILTY_MANAGER_H
#define _CAPABILTY_MANAGER_H

#include "bridge_processor.h"

#include <rapidjson/document.h> //todo:move to the cpps

#include <map>
#include <memory>
#include <string>

using rapidjson::Document;

class WebSession;

/** @class CapabilityManager
 * Base class defining the basic functionality of a CapabilityManager.
 *
 * @author Mostafa Gomaa
 */
class CapabilityManager
{
public:
	/** Constructor
   * @param capability_name the manager is intended to handle*/
	CapabilityManager(std::string capability_name)
	: capability_name_(capability_name), initialized_(false), finalized_(false)
	{
	}

	/** Destructor */
	~CapabilityManager()
	{
	}

	/** Implantation done by the extending classes.
  * Decode the JSON message and dispatches the request to the
  * intended operation provided by the intended BridgeProcessor
  * (the intended operation is encoded in the 'opcode' of the JSON
          message. the intended BridgeProcessor is denoted by the prefix
          of the topic_name of the JSON message)
  * @param d the Dom object containg the JSON message
  * @param session the session issuing the request.
  */
	virtual void
	handle_message(rapidjson::Document &d, std::shared_ptr<WebSession> session)
	{
	}

	/** Initialize the capability manager */
	virtual void
	init()
	{
		if (!initialized_) {
			initialized_ = true;
		}
	}

	/** Initialize the capability manager */
	virtual void
	finalize()
	{
		if (!finalized_) {
			while (!processores_.empty()) {
				processores_.begin()->second->finalize();
				processores_.erase(processores_.begin());
			}

			finalized_ = true;
		}
	}

	/** Implantation done by the extending classes.
   * Register a BridgeProcessor that extends the capability managed
   * by this capability manager, by creating a registry entry of the processor
   * mapped to its prefix.
   * @param processor ptr to the BridgeProcessor to register
   * @return true if the capability class was implemented by the processor
   */
	virtual bool
	register_processor(std::shared_ptr<BridgeProcessor> processor)
	{
		return false;
	}

	/** Get the name of the capability managed by this CapabilityManager
   * @return the internal name of the capability.
   */
	std::string
	get_name()
	{
		return capability_name_;
	}

protected:
	/// ProcessorMap is a map type, mapping Prefixes of BidgeProcessor to their
	/// instances ptrs
	typedef std::map<std::string, std::shared_ptr<BridgeProcessor>> ProcessorMap;

	ProcessorMap processores_;     /**< Registry to keep track or all processors and
                                the prefixes used to identify them */
	std::string  capability_name_; /**< Internal name of the capability managed by
                                   this CapabilityManager */
	bool         initialized_;     /**< Initialization status */
	bool         finalized_;       /**< Finalization status */
};

#endif