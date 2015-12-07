#include "generic_bridge.h"
#include "subscribe.h"


class FawkesBridge : public GenericBridge
{

	public:

		FawkesBridge(std::shared_ptr<Idispatcher> dispatcher)
		:GenericBridge(dispatcher){
			topics_prefix_="blackboard";
		}

		~FawkesBridge(){}

		bool init(){
			register_operation("subscribe",std::make_shared<Subscribe>());
			return true;
		}


		void incoming(std::string jsonStr){
			Document  d;
			deserialize(d,jsonStr);
				//std::map  <std::string,std::string> request_details;
			if (d.HasMember("op"))
			{
			    std::string op_name= std::string(d["op"].GetString());
			    // std::string msg_id= std::string(d["id"].GetString());
			    std::cout <<"The op name is: "<<op_name<<std::endl;			     
				   
			    std::string topic_fullname=std::string(d["topic"].GetString());

			    std::size_t pos = topic_fullname.find(topics_prefix_);
			     if (pos != std::string::npos)
			    {
			    	 std::string topic_name= topic_fullname.erase(pos 
			    	 	, topics_prefix_.length()+1);

			    	 d["topic"].SetString(StringRef(topic_name.c_str()));

			    	 std::cout <<"The topic name is: "<<topic_name<<std::endl;			     
			    }

				if(capabilities_.find(op_name)==capabilities_.end()){
					//throw exception or something
					 std::cout <<"No handler registerd for operation ("<<op_name<<")"<<std::endl;			     
					return;
				}

				capabilities_[op_name]->handle_message(d);

			}

		}


};