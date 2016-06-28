#ifndef __SERVICE_CAPABILITY_H_
#define __SERVICE_CAPABILITY_H_

#include <memory>

class ServiceCapability
{
	public:
		virtual void call_service( std::string srv_call_json , std::shared_ptr<WebSession> session ) = 0 ;
};

#endif