#ifndef _UTIL_SERIALIZER_
#define _UTIL_SERIALIZER_

#include <string>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>


using namespace rapidjson;

class Serializer
{
public:
	static std::string  op_subscribe(std::string prefiexed_topic_name 
		                              , std::string id    
		                              , std::string compression
		                              , unsigned int throttle_rate  
		                              , unsigned int queue_length   
		                              , unsigned int fragment_size)
	{
		std::string op="subscribe";

		StringBuffer s;
	  	Writer<StringBuffer> writer(s);      
		writer.StartObject();

		writer.String("op");
		writer.String(op.c_str(),(SizeType)op.length());

		writer.String("id");
		writer.String(id.c_str(),(SizeType)id.length());
		
		writer.String("topic");
		writer.String(prefiexed_topic_name.c_str(), (SizeType)prefiexed_topic_name.length());

		writer.String("throttle_rate");
		writer.Uint(throttle_rate);

		writer.String("queue_length");
		writer.Uint(fragment_size);

		writer.String("fragment_size");
		writer.Uint(queue_length);

		writer.String("compression");
		writer.String(compression.c_str(),(SizeType)compression.length());

		writer.EndObject();//End of complete Json_msg 
	    
	    //std::cout << s.GetString() << std::endl;

	    return s.GetString();
	}

};





#endif