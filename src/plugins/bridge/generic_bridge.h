#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/error/en.h>

#include <memory>
#include <string>
#include <iostream>
#include "ibridge.h"
#include "idispatcher.h"

using namespace rapidjson;

class GenericBridge : public Ibridge
{

public:

	GenericBridge(std::shared_ptr<Idispatcher> dispatcher);
	~GenericBridge();

	virtual void incoming(std::string jsonStr)=0;
	
	void outgoing(std::string jsonStr);

	bool deserialize(Document &d, std::string jsonStr);

	void serialize(std::string jsonStr);

		//overriding from IBridge
	bool init();

	void process_request(std::string jsonStr);


protected:
	std::string 					topics_prefix_;
	std::shared_ptr<Idispatcher> 	dispatcher_;

};