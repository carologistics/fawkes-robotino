#ifndef INTERFACE_BRIDGE_PROCESSOR_H
#define INTERFACE_BRIDGE_PROCESSOR_H

#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>


using namespace rapidjson;
using namespace std;

class IBridgeProcessor{

public:
	IBridgeProcessor(){}

	~IBridgeProcessor(){}

	bool subscribe(std::string topic_name){
		return true;

	}

	std::string read_topic(std::string topic_name){
			StringBuffer s;
	    Writer<StringBuffer> writer(s);
	    
	    writer.StartObject();
	    writer.String("hello");
	    writer.String("world");
	    writer.String("t");
	    writer.Bool(true);
	    writer.String("f");
	    writer.Bool(false);
	    writer.String("n");
	    writer.Null();
	    writer.String("i");
	    writer.Uint(123);
	    writer.String("pi");
	    writer.Double(3.1416);
	    writer.String("a");
	    writer.StartArray();
	    for (unsigned i = 0; i < 4; i++)
	        writer.Uint(i);
	    writer.EndArray();
	    writer.EndObject();

	    std::cout << s.GetString() << std::endl;

	    return s.GetString();

	}

};

#endif