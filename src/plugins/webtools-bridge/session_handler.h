
#ifndef _SESSION_TERMINATOR_H
#define _SESSIOM_TERMINATOR_H

class WebSession;

class SessionHandler
{
	public:
		
		//Just to be able to use it as a type in WebSession
		//TODO:: replace by dynamic_casting in the Websession side
		SessionHandler(){};
		~SessionHandler(){};

		virtual void session_terminated( std::shared_ptr <WebSession> session) {}; 

	protected:
		/*TODO:: have an interface "terminatable" and extend it by the WebSession.
		Include the interface here and implement the default registring behavior here*/
  		// virtual void register_as_terminat_listener( std::shared_ptr <WebSession> session ){};
  		// virtual void unregister_as_terminat_listener( std::shared_ptr <WebSession> session){};
  	
};

#endif