#ifndef _UTIL_SERIALIZER_
#define _UTIL_SERIALIZER_

#include <string>


class Serializer
{
public:
	static std::string  op_subscribe(std::string prefixed_topic_name 
		                              , std::string id    
		                              , std::string compression
		                              , unsigned int throttle_rate  
		                              , unsigned int queue_length   
		                              , unsigned int fragment_size);

	static std::string  op_unsubscribe(std::string prefixed_topic_name 
		                              , std::string id) ;

};

#endif