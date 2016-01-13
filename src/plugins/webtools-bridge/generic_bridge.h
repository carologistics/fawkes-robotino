#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/error/en.h>

#include <memory>
#include <string>
#include <map>
#include <iostream>
#include "interfaces/ibridge.h"
#include "interfaces/idispatcher.h"
#include "interfaces/icapability.h"

using namespace rapidjson;

class GenericBridge : public Ibridge
{

public:

	GenericBridge(std::shared_ptr<Idispatcher> dispatcher,std::string target_prefix);
	~GenericBridge();

	void incoming(std::string jsonStr);
	
	void outgoing(std::string jsonStr);

	bool deserialize(Document &d, std::string jsonStr);

	void serialize(std::string jsonStr);

		//overriding from IBridge
	bool init();

	void process_request(std::string jsonStr);


	/*adding/removing the capability that is responsible 
	for this operation to the map with the operation name as key*/
	void register_operation(std::string, std::shared_ptr<Icapability> capability);

	void deregister_operation(std::string, std::shared_ptr<Icapability> capability);


protected:
	//This is the prefix that is expected from the topics mapped to this bridge
	std::string 												target_prefix_;
	std::shared_ptr<Idispatcher> 								dispatcher_;
	std::map <std::string , std::shared_ptr<Icapability> > 		capabilities_ ;

};