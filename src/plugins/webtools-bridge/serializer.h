#ifndef _UTIL_SERIALIZER_
#define _UTIL_SERIALIZER_

#include <string>


class Serializer
{
public:
	static std::string  op_subscribe( std::string topic_name 
		                            , std::string id    
		                            , std::string compression
		                            , unsigned int throttle_rate  
		                            , unsigned int queue_length   
		                            , unsigned int fragment_size);

	static std::string  op_unsubscribe(std::string topic_name , std::string id);

	static std::string	op_advertise 	( std::string topic_name , std::string id , std::string type );
	static std::string	op_unadvertise	( std::string topic_name , std::string id );
	static std::string	op_publish		( std::string topic_name , std::string id , bool latch ,  std::string msg_in_json);
				
};

#endif