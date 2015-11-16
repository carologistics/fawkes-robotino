#include "web_session.h"
#include <core/threading/mutex.h>

using namespace fawkes;

	web_session::web_session()
	{
		data_mutex_= new fawkes::Mutex();
	}
	web_session::~web_session(){
		delete data_mutex_;
	}

	void
	web_session::set_connection_hdl(websocketpp::connection_hdl hdl){
		hdl_=hdl;
	}

	void 
	web_session::set_endpoint(websocketpp::lib::shared_ptr<server> endpoint_ptr){
		endpoint_ptr_=endpoint_ptr_;
	}

	void
	web_session::set_id(int id){
		session_id_=id;
	}

	void 
	web_session::set_name(std::string name){
		session_name_=name;
	}

	void
	web_session::set_status(std::string status){
		status_=status;
	}

	int 
	web_session::get_id(){
		return session_id_;
	}

	std::string 
	web_session::get_name(){
		return session_name_;
	}

	std::string 
	web_session::get_status(){
		return status_;
	}


	server::connection_ptr 
	web_session::get_connection_ptr(){
		return endpoint_ptr_->get_con_from_hdl(hdl_);
	}

	bool 
	web_session::send(std::string msg){
		data_mutex_->lock();
		websocketpp::lib::error_code ec;

		          std::cout << ">TO WEB::sending message: " << std::endl;
		endpoint_ptr_->send(hdl_, msg, websocketpp::frame::opcode::text, ec);

	        if (ec) {
	            std::cout << "> Error sending message: " << ec.message() << std::endl;
				data_mutex_->unlock();
	            return false;
	        }
			data_mutex_->unlock();
	        
	        return true;
    
	}



