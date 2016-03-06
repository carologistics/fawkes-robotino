#include "session_listener.h"
#include "web_session.h"


SessionListener::SessionListener()
{}

SessionListener::~SessionListener()
{}

void
SessionListener::session_terminated( std::shared_ptr <WebSession> session)
{}

void 
SessionListener::register_as_terminat_listener( std::shared_ptr <WebSession> session )
{
 	session->register_callback( WebSession::TERMINATE , shared_from_this() );
};
 
void
SessionListener::unregister_as_terminat_listener( std::shared_ptr <WebSession> session)
{
	session->unregister_callback( WebSession::TERMINATE , shared_from_this() );
};
