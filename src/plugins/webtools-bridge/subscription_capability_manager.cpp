#include "subscription_capability_manager.h"
#include "subscription_capability.h"

#include <core/exceptions/software.h>

using namespace fawkes;
using namespace rapidjson;


//DECIDE..will i extend the susbscribtion capability or not...i can even make i a virtual inheritance.
//but then the capability manager also became a capability...it will implement the operations anyways
//and i can even keep the Subscription list in the base class and be able to access it from the manager 
//and the processor...but Subscription then will contain all the list of any ever subscribed processor 
// at the very same place..



SubscriptionCapabilityManager::SubscriptionCapabilityManager()	
:	CapabilityManager("Subscription")
{

}

SubscriptionCapabilityManager::~SubscriptionCapabilityManager()
{
	//TODO: check if something needs to be changed
}
	

bool
SubscriptionCapabilityManager::register_processor(std::shared_ptr <BridgeProcessor> processor )
{
	std::shared_ptr <SubscriptionCapability> Subscription_processor;
	Subscription_processor = std::dynamic_pointer_cast<SubscriptionCapability> (processor);
	if(Subscription_processor == NULL)
	{
		return false;
	}
	//find if it was used before
	std::string processor_prefix= processor->get_prefix();
	
	if(processores_.find(processor_prefix) == processores_.end())
	{
		//throw exception this prefix name is invalide coz it was used before

	}

	processores_[processor_prefix]=processor;
	
	return true;
}

void
SubscriptionCapabilityManager::handle_message(Document &d
											, std::shared_ptr<WebSession> session)
{	

	//TODO::MOVE ALL THE TYPE RELATED STUFF TO a proper protocol Class
	if(!d.HasMember("topic") || !d.HasMember("id")){
		throw fawkes::MissingParameterException("SubscriptionCPM: Wrong Json Msg, topic name or id missing!");
	}

	//TODO::Pass the Dom the a Protocol Class that will have the deserialized types
	std::string msg_op = std::string(d["op"].GetString());

	std::string msg_topic = std::string(d["topic"].GetString());
	std::string match_prefix ="";

	//Find longest matching processor presfix within topic_name 
	//TODO:Check the logic..might be wrong
	//TODO:posible Optimization. if the same topic name exists in the topic_subscription just get the prefix from there
	for( ProcessorMap::iterator it = processores_.begin();
		it != processores_.end(); it++)
	{
		std::string processor_prefix = it->first;
		std::size_t found_at = msg_topic.find(processor_prefix,0);

		if(found_at != std::string::npos)
		{
			//allows freedom of 2 characters before the match to account 
			//for leading "/" ot "//" or and other startting charachters
			if (found_at <= 1)
			{
				if(processor_prefix.length() == match_prefix.length())
				{
					//Throw an exception. That there are 2 names with confusion prefixs
					//a conflict like "/bl" and  "bl1" when looking for "bl".
					//this cozed by the fliexibility
					throw fawkes::IllegalArgumentException("SubscriptionCPM: Could not select processor, 2 conflicting names!");

				}
				else if(processor_prefix.length() > match_prefix.length())
				{
					match_prefix = processor_prefix;
				}
			}
		}
	}

	if(match_prefix.length() == 0)
	{
		throw fawkes::IllegalArgumentException("SubscriptionCPM: No processor recognized for this topic!") ;
	}

	//Go with To the proper operation with the bridge_prefix and the request Paramters
	if(msg_op=="subscribe")
	{	
		std::string  topic_name 	= 	"";
		std::string  id 			= 	"";
		std::string  compression	=	"";	
		unsigned int throttle_rate	=	0;
		unsigned int queue_length 	=	1;
		unsigned int fragment_size 	=	0;
		
		if(d.HasMember("topic"))		 topic_name 	= 	std::string( d["topic"].GetString());
		if(d.HasMember("id")) 			 id 			= 	std::string(d["id"].GetString());
		if(d.HasMember("compression")) 	 compression	=	std::string(d["compression"].GetString());	
		if(d.HasMember("throttle_rate")) throttle_rate	=	d["throttle_rate"].GetUint();
		if(d.HasMember("queue_length"))  queue_length 	=	d["queue_length"].GetUint();
		if(d.HasMember("fragment_size")) fragment_size 	=	d["fragment_size"].GetUint();
		
		subscribe( match_prefix
				, topic_name , id , compression , throttle_rate , queue_length , fragment_size 
				, session);
	}else 

	if (msg_op=="unsubscribe")
	{
		std::string topic_name 		= 	std::string( d["topic"].GetString() );
		std::string id 				= 	std::string(d["id"].GetString());
	
		unsubscribe(match_prefix, topic_name, id , session);	
	}

}


void 
SubscriptionCapabilityManager::subscribe( std::string bridge_prefix
										, std::string topic_name 
										, std::string id 		
										, std::string compression
										, unsigned int throttle_rate	
										, unsigned int queue_length 	
										, unsigned int fragment_size 	
									   	, std::shared_ptr<WebSession> session)
{

	std::shared_ptr <SubscriptionCapability> processor;
	processor = std::dynamic_pointer_cast<SubscriptionCapability> (processores_[bridge_prefix]);
	//should be garanteed to work

	std::shared_ptr <Subscription> subscription;
	
	try{
		//always creates a new subscription for that topic with the given Session and parameters
		//TODO:: pass the pure string arguments or a "protocol" type
		subscription = processor-> subscribe(topic_name 
													, id 		
													, compression
													, throttle_rate	
													, queue_length 	
													, fragment_size 	
													, session);

	}catch(Exception &e){
		throw e;
	}

	/*push it to the topic_subscription_map maintaing only ONE Subscription instance per topic.
	*ps.Subscription contains all the clients maped to thier individuale requests*/

	//Is it a new topic? Just push it to the map and activate the Subscription

	//Mutex.lock()
	if( topic_Subscription_.find(topic_name) == topic_Subscription_.end() )
	{
		topic_Subscription_[topic_name] = subscription;
		//Activate the listeners or whatever that publishs
		subscription->activate();
	}else{
		topic_Subscription_[topic_name]->subsume(subscription);
	}

//I LOVE THIS LINE: 
	topic_Subscription_[topic_name]->add_request(id , compression , throttle_rate , queue_length , fragment_size , session);
	/*this line showed me so many things that lead me to a new restructuring a lot in my mind in my mind.
    So first moving this line from the processor to here eliminates the bug that i was struggling with. Since it only add_request
    to those subscriptions that is actually correct eliminating the need to make all the session registering for the temporary dormant
    Subscription object that is always returned by the processor.
    Moving it here only registers the subscription to the session [callbacks] only if this is the one worthy of keeping*/

    /*but then again what does it even mean to make a new temp subscription Object by the processor which only stores redundant data
    [Interface, Interface Listener to register to and blackboard interface] that u already had in the Active object (in case that was not the first subscription to this topic)
    It seems to me that we are (and we actually are) the processor to call its subscribe method to do some of processing processor specific.
    The looking for the interface, opening it, and passing it to a Subscription Object to store this is very redundant after the first time.

    MoreOver, Why am i storing a Subscription Object Separate from the Superscription Capability anyways. Why doest the bridge just extend
    Subscription Capability that knows all what happens on the subscription and just leaves a virtual Methods with the capability Operations name to specify the behaviour
    Why is The bridge processor even called a bridge processor and why does it "have" capability related objects. It sounds now that the capability Specific Object [subscription]
    could be the capability itself and the Bridge [aka Bridge Processor] itslef.

    Then i think about the implications and that we actually wont return anything to the Capability Manager which stores a list of bridges (also stored in every other capability Manager that this processor extend the capability Of).
    . why the hell a Bridge Manager is called a Bridge Manager and what deos it manage..U would expect it to manage bridges but that is a very vague concept on this design.
    What is a bridge in this analogy is it the processor. if so. then why is there a lot of things needs to be done outside of it

    Anyway...From here i thought a lot a reached also a stream of idea that make me think of the Client centred design that i think i should have.Lets start
    with the definition of the bridge from the clients side..Why do we have one session one client...blabla ...all the arguments are clear in mu head but i will put them in the
    commit when i start the re factoring..It makes not sense to me at this point to go like  this.
    Due to that intersecting parts all over and me trying to accommodate functionalities from the wrong directing and its hard to add or remove code.
    It is highly not clear to me what i did and why i did it and the readability of the code is really bad....If u set the right assumption from the start and follow the idea
    THEY intended the bridge to be with out trying to extend that idea..I think this design is awesome. Just remove all the levels of indirection

    Lets try this and if it worked with as much sessions as i want
    (meaning the bug is where i thought it was) instead of fixing the bug i will
    restructure the whole thing to element the redundancy and miss clarity. More explanations to come*/

	//Mutex.unlock();
}

void
SubscriptionCapabilityManager::unsubscribe	( std::string bridge_prefix
											, std::string topic_name 
											, std::string id 		
											, std::shared_ptr<WebSession> session)
{
	//select thre right processor
	std::shared_ptr <SubscriptionCapability> processor;
	processor = std::dynamic_pointer_cast<SubscriptionCapability> (processores_[bridge_prefix]);
	
	std::shared_ptr <Subscription> subscription;
	
	if(topic_Subscription_.find(topic_name) != topic_Subscription_.end()){

		subscription = topic_Subscription_[topic_name];

		try{
			processor->unsubscribe(id, subscription ,session );
		}catch (Exception &e){
			throw e;
		}

		if(subscription-> empty()){
			topic_Subscription_[topic_name] -> finalize();
			topic_Subscription_.erase(topic_name);
		}

		return;
	}
	
	//throw exception that topic was not found

}

