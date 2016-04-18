#include "serializer.h"

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

using namespace rapidjson;

std::string  
Serializer::op_subscribe(std::string prefixed_topic_name 
		                              , std::string id
		                              , std::string type 
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
	
	writer.String("type");
	writer.String(type.c_str(),(SizeType)type.length());
	
	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.String("throttle_rate");
	writer.Uint(throttle_rate);

	
	writer.String("queue_length");
	if(queue_length <1 ) 	 queue_length = 1;
	writer.Uint(queue_length);
	
	if(fragment_size != 0)
	{
		writer.String("fragment_size");
		writer.Uint(fragment_size);
	}

	writer.String("compression");
	writer.String(compression.c_str(),(SizeType)compression.length());

	writer.EndObject();//End of complete Json_msg 
    
    //std::cout << s.GetString() << std::endl;

    return s.GetString();
}

std::string  
Serializer::op_unsubscribe(std::string prefixed_topic_name 
		                              , std::string id)
{
	std::string op="unsubscribe";

	StringBuffer s;
  	Writer<StringBuffer> writer(s);      
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(),(SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(),(SizeType)id.length());
	
	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.EndObject();//End of complete Json_msg 
    
    //std::cout << s.GetString() << std::endl;

    return s.GetString();
}


std::string
Serializer::op_advertise( std::string prefixed_topic_name , std::string id , std::string type )
{
	std::string op="advertise";

	StringBuffer s;
  	Writer<StringBuffer> writer(s);      
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(),(SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(),(SizeType)id.length());
	
	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.String("type");
	writer.String(type.c_str(), (SizeType)type.length());

	writer.String("latch");
	writer.Bool(false);
	
	writer.String("queue_size");
	writer.Int(100);
	

	writer.EndObject();//End of complete Json_msg 
    
    //std::cout << s.GetString() << std::endl;

    return s.GetString();
}

std::string
Serializer::op_unadvertise( std::string prefixed_topic_name , std::string id )
{
	std::string op="unadvertise";

	StringBuffer s;
  	Writer<StringBuffer> writer(s);      
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(),(SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(),(SizeType)id.length());
	
	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.EndObject();//End of complete Json_msg 
    
    //std::cout << s.GetString() << std::endl;

    return s.GetString();
}

std::string
Serializer::op_publish( std::string prefixed_topic_name , std::string id 
						, bool latch, std::string msg_in_json)
{
	std::string op="publish";

	StringBuffer s;
  	Writer<StringBuffer> writer(s);      
	writer.StartObject();

	writer.String("op");
	writer.String(op.c_str(),(SizeType)op.length());

	writer.String("id");
	writer.String(id.c_str(),(SizeType)id.length());
	
	writer.String("topic");
	writer.String(prefixed_topic_name.c_str(), (SizeType)prefixed_topic_name.length());

	writer.String("latch");
	writer.Bool(latch);
	
	writer.String("msg");
	Document  d;

	if (d.Parse(msg_in_json.c_str()).HasParseError())
	{
		//std::cout<< GetParseError_En(d.GetParseError());
		return "";
		//throw
	}
	d.Parse(msg_in_json.c_str());
	d.Accept(writer);

	writer.EndObject();//End of complete Json_msg 
    
    //std::cout << s.GetString() << std::endl;

    return s.GetString();
}

// Serialier::append_json(	Writer<StringBuffer> &writer, std::string json_str)
// {
// 	Document  d;

// 	if (d.Parse(json_str.c_str()).HasParseError())
// 	{
// 		//std::cout<< GetParseError_En(d.GetParseError());
// 		return "";
// 		//throw
// 	}
// 	d.Parse(json_str.c_str());
// 	d.Accept(writer);    
// }

std::string
Serializer::serialize(CLIPS::Fact::pointer fact)
{

	StringBuffer s;
  	Writer<StringBuffer> writer(s);      
	writer.StartObject();

    std::string name = fact->get_template()->name();
	writer.String("name");//key for fact_name 
	writer.String(name.c_str(),(SizeType)name.length());

	std::vector<std::string> slot_names= fact->slot_names();
	if(slot_names.size() > 0 )// is it a non-ordered fact 
	{
		for(std::vector<std::string>::iterator it = slot_names.begin(); slot_names.size() > 0 && it != slot_names.end(); it++)
		{
			writer.String( it->c_str() , (SizeType) it->length() );//write slot name as a key for json pair
			
			CLIPS::Values values= fact->slot_value( *it );  
			bool multifield= values.size()>0; 
			if(multifield)     writer.StartArray();	// write values of slot as a json array only if slot is mutifeild
			for(CLIPS::Values::iterator it2 = values.begin(); it2 != values.end(); it2++)
			{
				switch(it2->type())
				  {
				  case CLIPS::TYPE_INTEGER :
				   writer.Int(it2->as_integer());
				  case CLIPS::TYPE_FLOAT :
				   writer.Double(it2->as_float());
				  default :
				    writer.String( it2-> as_string().c_str() , (SizeType)it2->as_string().length() );
				}
			}
			if(multifield)     writer.EndArray();
		}
	}else
	{//ordered fact
		writer.String("fields" );//wirte the json pair key that will be used to refrence the feilds of the fact
		CLIPS::Values values= fact->slot_value("");
		writer.StartArray();	//field values will be stored in a json array
		for(CLIPS::Values::iterator it = values.begin(); it != values.end(); it++)
		{
			switch(it->type())
			  {
			  case CLIPS::TYPE_INTEGER :
			   writer.Int(it->as_integer());
			  case CLIPS::TYPE_FLOAT :
			   writer.Double(it->as_float());
			  default :
			    writer.String( it-> as_string().c_str() , (SizeType)it->as_string().length() );
			}
		}
		writer.EndArray();
	}

	writer.EndObject();


    return s.GetString();
}


