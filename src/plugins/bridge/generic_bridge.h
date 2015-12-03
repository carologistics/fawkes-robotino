#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/error/en.h>

#include <memory>
#include <string>
#include <iostream>
#include "ibridge.h"
#include "idispatcher.h"
#include <boost/shared_ptr.hpp>

using namespace rapidjson;

class GenericBridge : public Ibridge
{

public:

	GenericBridge(boost::shared_ptr<Idispatcher> dispatcher)
	:dispatcher_(dispatcher)
	{

	}

	~GenericBridge();

	virtual void incoming(std::string jsonStr)=0;



	//forwards the outgoing msg to the web client 
	void outgoing(std::string jsonStr){
		dispatcher_->send_to_web(jsonStr);
	}



	bool deserialize(Document &d, std::string jsonStr){
		const char* json = jsonStr.c_str();
		d.Parse(json);

		if (d.Parse(json).HasParseError()) {
			std::cout<< GetParseError_En(d.GetParseError());
			return false;
		}

		return true;
	}

	void serialize(std::string jsonStr);


protected:
	std::string 					bridge_target;
	boost::shared_ptr<Idispatcher> 	dispatcher_;

};