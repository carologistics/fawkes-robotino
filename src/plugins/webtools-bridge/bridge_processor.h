/***************************************************************************
 * bridge_processor.h - Interface For Processors of Bridge Requests
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

#ifndef _BRIDGE_PROCESSOR_H
#define _BRIDGE_PROCESSOR_H

#include <string>

/** @class BridgeProcessor
 * Abstract class, defines the core functionalities needed by a BridgeProcessor
 * Conceptually, a bridge processor contains domain specific implementations to
 * the capabilities it has.
 *
 * @author Mostafa Gomaa
 */
class BridgeProcessor
{
public:
	/** Constructor
   * @param prefix used to identity this processor and to rout requests to it.
   */
	BridgeProcessor(std::string prefix) : prefix_(prefix), initialized_(false), finalized_(false)
	{
	}

	/** Destructor */
	virtual ~BridgeProcessor()
	{
	}

	/** Initialize the BridgeProcessor instance. */
	virtual void
	init()
	{
		if (!initialized_) {
			initialized_ = true;
		}
	}

	/** finalize the BridgeProcessor instance. */
	virtual void
	finalize()
	{
		if (!finalized_) {
			finalized_ = true;
		}
	}

	/** Get the internal prefix identifying this BridgeProcessor
   * @return the internal prefix that is used to identify this processor
   */
	std::string
	get_prefix()
	{
		return prefix_;
	}

protected:
	std::string prefix_;      /**< Internal name of the bridge processor used as topics
                          prefix to refer to this processor */
	bool        initialized_; /**< Initialization status */
	bool        finalized_;   /**< Finalization status */
};

#endif
