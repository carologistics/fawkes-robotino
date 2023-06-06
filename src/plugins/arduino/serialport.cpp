/***************************************************************************
 *  serialport.cpp - Serial Port handler
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

#include "serialport.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

using serial_port_base = boost::asio::serial_port_base;

SerialPort::SerialPort(const char                                *port,
                       boost::function<void(const std::string &)> receive_callback,
                       std::shared_ptr<std::mutex>                port_mutex,
                       unsigned int                               baud_rate,
                       const char                                *start_of_command,
                       const char                                *end_of_command)
{
	receive_callback_ = receive_callback;

	mutex_ = port_mutex;
	std::lock_guard<std::mutex> lock(*mutex_);

	start_of_command_ = (char*)start_of_command;
	end_of_command_   = (char*)end_of_command;


	boost::system::error_code ec;

	port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
	port_->open(port, ec);
	if (ec) {
		throw ec;
	}

	// option settings...
	port_->set_option(serial_port_base::baud_rate(baud_rate));
	port_->set_option(serial_port_base::character_size(8));
	port_->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
	port_->set_option(serial_port_base::parity(serial_port_base::parity::none));
	port_->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));

	async_read_some_();
}

void SerialPort::async_read_some_()
{
	if (port_.get() == NULL || !port_->is_open()) return;

	port_->async_read_some(
		boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
		boost::bind(
			&SerialPort::on_receive_,
			this, boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void SerialPort::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
{
	boost::mutex::scoped_lock look(*mutex_);

	if (port_.get() == NULL) return;

	if(!port_) this->~SerialPort(); //If port is closed then just dissolve this instance
	if (ec) {
		async_read_some_();
		return;
	}

	int starts_at = -1;
	int ends_at = -1;
	for (unsigned int i = 0; i < bytes_transferred; ++i) {
		bool is_starting = true;
		for(unsigned int j = 0; j < std::strlen(start_of_command_) && j + i < bytes_transferred; j++){
			char c = read_buf_raw_[i + j];
			if(c != start_of_command_[j]) {
				is_starting = false;
				break;
			}
		}
		read_buf_raw_
		if(is_starting){
			starts_at = i;
			break;
		}
	}
	if(starts_at == -1) {
		return;
	}

	for (unsigned int i = starts_at + std::strlen(start_of_command_); i < bytes_transferred; ++i) {
		bool is_ending = true;
		for(unsigned int j = 0; j < std::strlen(end_of_command_) && j + i < bytes_transferred; j++){
			char c = read_buf_raw_[i + j];
			if(c != end_of_command_[i]){
				is_ending = false;
				break;
			}
		}
		if(is_ending) {
			ends_at = i;
			break;
		}
	}
	if(starts_at == -1) {
		return;
	}

	read_buf_str_ = std::string(read_buf_raw_);
			this->on_receive_(read_buf_str_);
			read_buf_str_.clear();
		}
    else
    {
	    read_buf_str_ += c;
    }
  }

	async_read_some_();
  }


void SerialPort::on_receive_(const std::string &data)
{
	//TODO
}


SerialPort::~SerialPort() {
	port_->close();
	mutex_->unlock();
}
