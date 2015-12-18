#ifndef INTERFACE_BRIDGE_PROCESSOR_H
#define INTERFACE_BRIDGE_PROCESSOR_H

class IBridgeProcessor{

public:
	IBridgeProcessor(){}

	~IBridgeProcessor(){}

	virtual bool subscribe(std::string topic_name)=0;
	virtual std::string read_single_topic(std::string topic_name)=0;

};

#endif
