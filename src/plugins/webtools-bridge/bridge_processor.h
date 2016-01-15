#ifndef _BRIDGE_PROCESSOR_H
#define _BRIDGE_PROCESSOR_H

#include <string>

class BridgeProcessor{

public:
	BridgeProcessor()
	:	prefix_(prefix)
	{

	}
	
	~BridgeProcessor();

	std::string get_prefix()
	{
		return prefix;
	}

private:
	std::string prefix;

};

#endif
