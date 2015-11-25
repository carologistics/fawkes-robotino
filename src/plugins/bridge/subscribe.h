#include <map>


class Subscribe{
	public:
		Subscribe();
		~Subscribe();

		void subscirbe(std::string topic,std::string client_id);
		void unsubscirbe(std::string topic, std::string client_id);

		void publish(std::string topic, std::string client_id );

	private:
		std::map <std::string,std::string> subscirbtions;

}