#ifndef INTERFACE_DISPATCHER_H
#define INTERFACE_DISPATCHER_H

class Idispatcher{
	public:
		virtual bool send_to_web(std::string msg)=0;

};


#endif
