/***************************************************************************
 *  com_thread.cpp - Arduino com thread
 *
 *  Created:  Tue 6 Jun 15:57:10 2023
 *  Copyright:	2023 Tim Wendt		
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef _SERIALPORT_H
#define _SERIALPORT_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <memory>
#include <string>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

#define SERIAL_PORT_READ_BUF_SIZE 256

class SerialPort
{
protected:
	bool terminate_thread = false;
	boost::thread serial_service_thread_;
	boost::asio::io_service       io_service_;
	serial_port_ptr               port_;
	// std::shared_ptr<boost::mutex> mutex_;
	boost::mutex mutex_;

	char        read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
	std::string read_buf_str_;

	std::string start_of_command_;
	std::string end_of_command_;

	boost::function<void(const std::string &)> receive_callback_;
	boost::function<void()> deconstruct_callback_;
	bool                                       has_started = false;

public:
	SerialPort(std::string                                port,
	           boost::function<void(const std::string &)> receive_callback,
               boost::function<void()> deconstruct_callback,
	           // std::shared_ptr<boost::mutex>              mutex,
	           unsigned int                               baud_rate        = 115200,
	           std::string                                start_of_command = "AT ",
	           std::string                                end_of_command   = "+");
	~SerialPort();
	bool write(const std::string &buf);
	bool write(const char *buf, const int &size);

private:
	void         async_read_some_();
	virtual void on_receive_(const boost::system::error_code &ec, size_t bytes_transferred);
};

#endif /* _SERIALPORT_H */
