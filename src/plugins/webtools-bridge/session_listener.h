
#ifndef _SESSION_LISTENER_H
#define _SESSION_LISTENER_H

#include <memory>

class WebSession;

class SessionListener
: public std::enable_shared_from_this<SessionListener>
{
	public:
		
		//TODO:: replace by dynamic_casting in the Websession side
		SessionListener();
		~SessionListener();

		virtual void session_terminated( std::shared_ptr <WebSession> session) ; 

	protected:
		/*TODO:: have an interface "terminatable" and extend it by the WebSession.
		Include the interface here and implement the default registring behavior here*/
  		 void register_as_terminat_listener( std::shared_ptr <WebSession> session );
  		 void unregister_as_terminat_listener( std::shared_ptr <WebSession> session );
  	
};

#endif