#include <map>
#include <isession.h>


class web_session : public Isession
{
private:
    int                                        		 session_id_;
    std::string                              		 session_name_;
    websocketpp::lib::shared_ptr<server>       		 endpoint_ptr_;
    websocketpp::connection_hdl                		 hdl_;
    std::string						 status_;

public:
	web_session();
	~web_session();

    	std::map<std::string,std::string>	http_req;

	void set_connection_hdl(websocketpp::connection_hdl hdl);
	void set_endpoint(websocketpp::lib::shared_ptr<server> endpoint_ptr);
	void set_id(int id);
	void set_name(std::string name);
	void set_status(std::string status);

	int 			get_id();
	std::string 		get_name();
	std::string 		get_status();
	server::connection_ptr 	get_connection_ptr();

	void send(std::string msg);



};

