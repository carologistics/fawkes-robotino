#ifndef INTERFACE_BRIDGE_MANAGER_H
#define INTERFACE_BRIDGE_MANAGER_H

class IbridgeManager{
	public:
		virtual bool subscribe(std::string topic_name)=0;

		virtual bool publish(std::string topic_name)=0;
};

#endif