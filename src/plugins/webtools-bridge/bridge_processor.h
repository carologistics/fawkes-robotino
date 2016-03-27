#ifndef _BRIDGE_PROCESSOR_H
#define _BRIDGE_PROCESSOR_H

#include <string>


class BridgeProcessor{

public:
	BridgeProcessor(std::string prefix)
	:	prefix_(prefix)
	{
	}
	
	virtual ~BridgeProcessor(){}

	virtual void init()
	{}
	

	std::string get_prefix()
	{
		return prefix_;
	}

protected:
	std::string prefix_;



};

#endif		
