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

class BridgeProcessor{

public:
	BridgeProcessor(std::string prefix)
	:	prefix_(prefix)
	,	initialized_(false)
	,	finalized_(false)
	{
	}
	
	virtual ~BridgeProcessor(){}

	virtual void init()	
	{
		if(!initialized_)
		{
			initialized_ = true;
		}
	}

	virtual void finalize()	
	{
		if(!finalized_)
		{
			finalized_ = true;
		}
	}


	std::string get_prefix()
	{
		return prefix_;
	}

protected:
	std::string prefix_;
	bool initialized_;
	bool finalized_;

};

#endif		
