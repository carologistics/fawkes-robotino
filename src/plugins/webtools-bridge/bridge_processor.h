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
