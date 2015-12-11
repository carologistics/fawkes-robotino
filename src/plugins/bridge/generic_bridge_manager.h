#include  "interfaces/ibridge_manager.h"

class GenericBridgeManager : public  IbridgeManager
{
	public:
		GenericBridgeManager();

		~GenericBridgeManager();

		//subscribe();

		void publish();

	private:

		//Bridge_processor 	Handle the Single Requests from Distenation_Resource
		//Generic_Bridge	Handles all bookkeeping of incoming requestes and forward outgoing ones

};