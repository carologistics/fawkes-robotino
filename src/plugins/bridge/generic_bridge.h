#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/error/en.h>

#include <ibridge.h>
#include <idispatcher.h>


class GenericBridge: public ibridge
{

public:

	GenericBridge(std::shared_ptr<Idispatcher> dispatcher)
	:dispatcher_(dispatcher)
	{

	}

	~GenericBridge();

	virtual void incoming(std::msg jsonStr)=0;

	//forwards the outgoing msg to the web client 
	void outgoing(std::msg jsonStr){
		dispatcher_.send_to_web(jsonStr);
	}

	Documnet serialize(std::msg jsonStr){
		const char* json = jsonStr.c_str();
		Document d;
		d.Parse(json);

		if (d.Parse(json).HasParseError()) {
			std::cout<< GetParseError_En(d.GetParseError());
		}
		
		return d;
	}

	void deserialize(std::msg jsonStr);


private:
	std::shared_ptr<Idispatcher>  dispatcher_;

}