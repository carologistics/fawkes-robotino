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

	void outgoing(std::msg jsonStr);

	void serialize(std::msg jsonStr);

	void deserialize(std::msg jsonStr);


private:
	std::shared_ptr<Idispatcher>  dispatcher_;

}