class SubscribtionCapabilityManager
: public CapabilityManger
{

public:

private:
	void subscribe();
	void unsubscribe();

	std::map <std::string,std::shared_ptr<Subscribtion> > topic_subscribtion_;
};