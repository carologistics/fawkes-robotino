#include <list>

struct  client{
	std::string subscribtion_id;
   std::string throttle_rate;
   std::string queue_length;
   std::string fragment_size;
   // compression": compression

};

class Subscribtion{
	
	public:
		Subscribtion(std::string client_id, std::string topic);
		~Subscribtion();

		void subscribe();
		void unsubscribe();

		void publish();

	private:
		std::string 			client_id;
		std::string 			topic;
		//handler 		 publish			find a way to register it
		std::list<client> 					client_detial;

}