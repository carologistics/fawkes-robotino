#ifndef _UTIL_SERIALIZER_
#define _UTIL_SERIALIZER_

#include <string>

namespace rapidjson{
	class writer,
	class GenericStringBuffer
}

class Serializer
{
public:
	static std::string  op_subscribe(std::string prefiexed_topic_name 
		                              , std::string id    
		                              , std::string compression
		                              , unsigned int throttle_rate  
		                              , unsigned int queue_length   
		                              , unsigned int fragment_size);

};

#endif