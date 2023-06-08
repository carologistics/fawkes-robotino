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

#include <boost/bind/bind.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>
#include <cstdint>

using serial_port_base = boost::asio::serial_port_base;

SerialPort::SerialPort(std::string                                port,
                       boost::function<void(const std::string &)> receive_callback,
                       unsigned int                               baud_rate,
                       std::string                                start_of_command,
                       std::string                                end_of_command)
{
	receive_callback_ = receive_callback;

	start_of_command_ = start_of_command;
	end_of_command_   = end_of_command;

	boost::system::error_code ec;

	port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
	port_->open(port.c_str(), ec);
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

void
SerialPort::async_read_some_()
{
	if (port_.get() == NULL || !port_->is_open())
		return;
	port_->async_read_some(boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
	                       boost::bind(&SerialPort::on_receive_,
	                                   this,
	                                   boost::asio::placeholders::error,
	                                   boost::asio::placeholders::bytes_transferred));
}

bool
start_with_pattern(const char *pattern, const char buf[], size_t bytes_transferred, size_t offset)
{
	for (int i = 0; i < strlen(pattern); ++i) {
		char c = buf[i + offset];
		if (c != pattern[i])
			return false;
	}
	return true;
}

bool
SerialPort::write(const std::string &buf)
{
	return write(buf.c_str(), buf.size());
}

bool
SerialPort::write(const char *buf, const int &size)
{
	boost::system::error_code ec;
	if (!port_)
		return false;
	if (size == 0)
		return false;
	return port_->write_some(boost::asio::buffer(buf, size), ec);
}

inline bool check_checksum(int offset, std::string checksum_str, char buf[], size_t bytes_transferred) {
	if(offset > 250) {
		return false;
	}
	bool state = false;
	for(int i = 0; i < checksum_str.size() && i + offset < bytes_transferred; ++i) {
		state = true;
		if(checksum_str[i] != buf[i + offset]) {
			return false;
		}
	}
	//state garantees to be false when for never runs only once
	return state;
}

std::string checksum(const std::string buf) {
	uint8_t sum = 0;
	for(uint8_t b = 0; b< buf.size(); b++)
	{
		sum += buf[b];
	}
	uint8_t low_byte = sum & 0xff;
	return std::to_string(low_byte);
}

void
SerialPort::on_receive_(const boost::system::error_code &ec, size_t bytes_transferred)
{
	boost::mutex::scoped_lock lock(mutex_);
	//TODO check for active port and request messsage
	if (port_.get() == NULL)
		return;

	if (!port_)
		this->~SerialPort(); //If port is closed then just dissolve this instance
	if (ec) {
		async_read_some_();
		return;
	}

	bool has_started = false;
	for (unsigned int i = 0; i < bytes_transferred; ++i) {
		bool is_starting =
		  start_with_pattern(start_of_command_.c_str(), read_buf_raw_, bytes_transferred, i);
		bool is_ending =
		  start_with_pattern(end_of_command_.c_str(), read_buf_raw_, bytes_transferred, i);

		if (has_started || is_starting) {
			has_started = true;
		}

		if(has_started && is_starting) {
			//last message is mixed up with new
			read_buf_str_.clear();
		}

		if (has_started) {
			read_buf_str_ += read_buf_raw_[i];
		}

		if (has_started && is_ending) {
			has_started = false;
			for (int j = 1; j < end_of_command_.size(); ++j) {
				read_buf_str_ += read_buf_raw_[i + j];
			}
			std::string checksum_ = checksum(read_buf_str_);
			if(check_checksum(i + end_of_command_.size(), checksum_, read_buf_raw_, bytes_transferred)){
				receive_callback_(read_buf_str_);
			}
			read_buf_str_.clear();
		}
	}

	async_read_some_();
}

SerialPort::~SerialPort()
{
	port_->cancel();
	port_->close();
	port_.reset();
	io_service_.stop();
	io_service_.reset();
}
