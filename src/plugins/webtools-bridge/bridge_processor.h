#ifndef _BRIDGE_PROCESSOR_H
#define _BRIDGE_PROCESSOR_H

#include <string>

class BridgeProcessor{

public:
	BridgeProcessor(std::string prefix)
	:	prefix_(prefix)
	{
	}
	
	virtual ~BridgeProcessor();

	std::string get_prefix()
	{
		return prefix_;
	}

private:
	std::string prefix_;

};

#endif
