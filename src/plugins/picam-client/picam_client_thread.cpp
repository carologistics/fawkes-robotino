/***************************************************************************
 *  picam_client_thread.cpp - Thread to subscribe to camera streams from the
 *                            raspberry pi camera and publish the images to
 *                            shared memory buffers.
 *
 *  Created: Sun Jun 30 17:36:00 2024
 *  Copyright  2024 Daniel Swoboda
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

#include "picam_client_thread.h"

#include "base64.h"

#include <aspect/logging.h>
#include <interfaces/ObjectTrackingInterface.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>

using namespace fawkes;
using namespace cv;
using namespace dnn;

#define HEADER_SIZE_1 20               // Header size for message of type 1
#define HEADER_SIZE_2 20               // Header size for message of type 2
#define HEADER_SIZE_3 32               // Header size for message of type 3
#define CONTROL_HEADER_SIZE 9          // Header size for control message
#define CONTROL_HEADER_SIZE_PAYLOAD 13 // Header size for control message
#define CONFIGURE_MESSAGE_SIZE 65      // Header size for control message

#define SLEEP_INTERVAL 1
#define DISCONNECT_THRESHOLD 10
#define RECONNECT_INTERVAL 5

/** @class PicamClientThread "picam_client_thread.h"
 * @author Daniel Swoboda
 */

/** Constructor. */
PicamClientThread::PicamClientThread()
: Thread("PicamClientThread", Thread::OPMODE_CONTINUOUS),
  BlackBoardInterfaceListener("PicamClientThread")
{
}

void
PicamClientThread::init()
{
	logger->log_info(name(), "Initializing Picam Client");
	logger->log_info(name(), ("OpenCV version " + std::string(CV_VERSION)).c_str());

	// get params for connection
	server_ip_     = config->get_string("plugins/picam_client/ip");
	server_port_   = config->get_int("plugins/picam_client/port");
	camera_width_  = config->get_int("plugins/picam_client/camera_intrinsics/width");
	camera_height_ = config->get_int("plugins/picam_client/camera_intrinsics/height");

	//shared memory buffer, initialise dynamically once the first image is received
	shm_id_         = config->get_string("plugins/object_tracking/buffer/shm_image_id");
	shm_buffer_     = new firevision::SharedMemoryImageBuffer(shm_id_.c_str(),
                                                        firevision::BGR,
                                                        camera_width_,
                                                        camera_height_);
	shm_id_res_     = config->get_string("plugins/object_tracking/buffer/shm_image_id_res");
	shm_buffer_res_ = new firevision::SharedMemoryImageBuffer(shm_id_res_.c_str(),
	                                                          firevision::BGR,
	                                                          camera_width_,
	                                                          camera_height_);

	bb_interface_ = blackboard->open_for_writing<PiCamPluginInterface>("PiCamPluginInterface");
}

void
PicamClientThread::loop()
{
	if (disconnect_counter_ > DISCONNECT_THRESHOLD) {
		connected_          = false;
		disconnect_counter_ = 0;
	}
	if (connected_) {
		data_.clear();

		// read the message type
		data_.resize(1);
		if (!receive_data(sockfd_, data_.data(), 1)) {
			logger->log_error(name(), "No message received in iteratione");

			std::this_thread::sleep_for(std::chrono::seconds(SLEEP_INTERVAL));
			disconnect_counter_ += 1;
			return;
		}
		disconnect_counter_ = 0;
		uint8_t message_type;
		std::memcpy(&message_type, &data_[0], 1);

		if (message_type == 1 || message_type == 2) {
			data_.clear();

			// read header for type 1 and 2 messages
			data_.resize(HEADER_SIZE_1);
			if (!receive_data(sockfd_, data_.data(), HEADER_SIZE_1)) {
				logger->log_error(name(), "Failed to read header");
				std::this_thread::sleep_for(std::chrono::seconds(SLEEP_INTERVAL));
				disconnect_counter_ += 1;
				return;
			}
			disconnect_counter_ = 0;
			uint64_t timestamp;
			uint32_t width, height, length;

			std::memcpy(&timestamp, &data_[0], 8);
			std::memcpy(&height, &data_[8], 4);
			std::memcpy(&width, &data_[12], 4);
			std::memcpy(&length, &data_[16], 4);

			timestamp                = ntohl(timestamp);
			uint64_t timestamp_secs  = timestamp / 1000000000;
			uint64_t timestamp_usecs = timestamp - timestamp_secs * 1000000000;
			height                   = ntohl(height);
			width                    = ntohl(width);
			length                   = ntohl(length);

			// read the frame payload based on the length
			std::vector<char> image_data(length);
			if (!receive_data(sockfd_, image_data.data(), length)) {
				logger->log_error(name(), "Failed to read image data");
				std::this_thread::sleep_for(std::chrono::seconds(SLEEP_INTERVAL));
				disconnect_counter_ += 1;
				return;
			}
			disconnect_counter_ = 0;

			// decode base64
			std::string base64_image(image_data.begin(), image_data.end());
			std::string decoded_image = base64_decode(base64_image);

			// decode image using OpenCV
			std::vector<uchar> img_data(decoded_image.begin(), decoded_image.end());
			cv::Mat            img = cv::imdecode(img_data, cv::IMREAD_COLOR);

			if (img.empty()) {
				logger->log_error(name(), "Failed to decode image");
				std::this_thread::sleep_for(std::chrono::seconds(SLEEP_INTERVAL));
				disconnect_counter_ += 1;
				return;
			}
			disconnect_counter_ = 0;

			// write received image in the right shared memory buffer, based on type
			if (message_type == 1) {
				firevision::convert(firevision::BGR,
				                    firevision::BGR,
				                    img.data,
				                    shm_buffer_->buffer(),
				                    camera_width_,
				                    camera_height_);
				shm_buffer_->set_capture_time(timestamp_secs, timestamp_usecs);
			}
			if (message_type == 2) {
				firevision::convert(firevision::BGR,
				                    firevision::BGR,
				                    img.data,
				                    shm_buffer_res_->buffer(),
				                    camera_width_,
				                    camera_height_);
				shm_buffer_res_->set_capture_time(timestamp_secs, timestamp_usecs);
			}

		} else if (message_type == 3) {
			data_.clear();
			// read the message header of type 3
			data_.resize(HEADER_SIZE_3);
			if (!receive_data(sockfd_, data_.data(), HEADER_SIZE_3)) {
				logger->log_error(name(), "Failed to read header");
				std::this_thread::sleep_for(std::chrono::seconds(SLEEP_INTERVAL));
				disconnect_counter_ += 1;
				return;
			}
			disconnect_counter_ = 0;

			// unpack the data
			uint64_t timestamp;
			uint32_t cls;
			float    x, y, h, w, acc;
			std::memcpy(&timestamp, &data_[0], 8);
			std::memcpy(&x, &data_[8], 4);
			std::memcpy(&y, &data_[12], 4);
			std::memcpy(&h, &data_[16], 4);
			std::memcpy(&w, &data_[20], 4);
			std::memcpy(&acc, &data_[24], 4);
			std::memcpy(&cls, &data_[28], 4);

			// convert to host byte order
			timestamp                = ntohll(timestamp);
			uint64_t timestamp_secs  = timestamp / 1000000000;
			uint64_t timestamp_usecs = timestamp - timestamp_secs * 1000000000;
			x                        = ntohlf(x);
			y                        = ntohlf(y);
			h                        = ntohlf(h);
			w                        = ntohlf(w);
			acc                      = ntohlf(acc);
			cls                      = ntohl(cls);

			if (timestamp > last_msg_time_) {
				last_msg_time_ = timestamp;
				msg_counter_   = 0;
				reset_interface();
			}

			// write the data to the blackboard
			write_to_interface(msg_counter_, x, y, h, w, acc, cls, timestamp_secs, timestamp_usecs);
			msg_counter_ += 1;
		}
	} else {
		if (connect_to_server() < 0) {
			connected_ = false;
			logger->log_error(name(), "Attempting to reconnect");
			std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_INTERVAL));
			return;
		} else {
			logger->log_warn(name(), "Connected to server");
			connected_ = true;
		}
	}
}

void
PicamClientThread::reset_interface()
{
	float    null_array[6]     = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	uint64_t null_timestamp[2] = {0, 0};
	bb_interface_->set_bbox_0(null_array);
	bb_interface_->set_bbox_0_timestamp(null_timestamp);
	bb_interface_->set_bbox_1(null_array);
	bb_interface_->set_bbox_1_timestamp(null_timestamp);
	bb_interface_->set_bbox_2(null_array);
	bb_interface_->set_bbox_2_timestamp(null_timestamp);
	bb_interface_->set_bbox_3(null_array);
	bb_interface_->set_bbox_3_timestamp(null_timestamp);
	bb_interface_->set_bbox_4(null_array);
	bb_interface_->set_bbox_4_timestamp(null_timestamp);
	bb_interface_->set_bbox_5(null_array);
	bb_interface_->set_bbox_5_timestamp(null_timestamp);
	bb_interface_->set_bbox_6(null_array);
	bb_interface_->set_bbox_6_timestamp(null_timestamp);
	bb_interface_->set_bbox_7(null_array);
	bb_interface_->set_bbox_7_timestamp(null_timestamp);
	bb_interface_->set_bbox_8(null_array);
	bb_interface_->set_bbox_8_timestamp(null_timestamp);
	bb_interface_->set_bbox_9(null_array);
	bb_interface_->set_bbox_9_timestamp(null_timestamp);
	bb_interface_->set_bbox_10(null_array);
	bb_interface_->set_bbox_10_timestamp(null_timestamp);
	bb_interface_->set_bbox_11(null_array);
	bb_interface_->set_bbox_11_timestamp(null_timestamp);
	bb_interface_->set_bbox_12(null_array);
	bb_interface_->set_bbox_12_timestamp(null_timestamp);
	bb_interface_->set_bbox_13(null_array);
	bb_interface_->set_bbox_13_timestamp(null_timestamp);
	bb_interface_->set_bbox_14(null_array);
	bb_interface_->set_bbox_14_timestamp(null_timestamp);
	bb_interface_->write();
}

void
PicamClientThread::write_to_interface(int      slot,
                                      float    x,
                                      float    y,
                                      float    h,
                                      float    w,
                                      float    acc,
                                      float    cl,
                                      uint64_t timestamp_secs,
                                      uint64_t timestamp_usecs)
{
	float    values[6]    = {x, y, h, w, acc, cl};
	uint64_t timestamp[2] = {timestamp_secs, timestamp_usecs};
	if (slot == 0) {
		bb_interface_->set_bbox_0(values);
		bb_interface_->set_bbox_0_timestamp(timestamp);
	} else if (slot == 1) {
		bb_interface_->set_bbox_1(values);
		bb_interface_->set_bbox_1_timestamp(timestamp);
	} else if (slot == 2) {
		bb_interface_->set_bbox_2(values);
		bb_interface_->set_bbox_2_timestamp(timestamp);
	} else if (slot == 3) {
		bb_interface_->set_bbox_3(values);
		bb_interface_->set_bbox_3_timestamp(timestamp);
	} else if (slot == 4) {
		bb_interface_->set_bbox_4(values);
		bb_interface_->set_bbox_4_timestamp(timestamp);
	} else if (slot == 5) {
		bb_interface_->set_bbox_5(values);
		bb_interface_->set_bbox_5_timestamp(timestamp);
	} else if (slot == 6) {
		bb_interface_->set_bbox_6(values);
		bb_interface_->set_bbox_6_timestamp(timestamp);
	} else if (slot == 7) {
		bb_interface_->set_bbox_7(values);
		bb_interface_->set_bbox_7_timestamp(timestamp);
	} else if (slot == 8) {
		bb_interface_->set_bbox_8(values);
		bb_interface_->set_bbox_8_timestamp(timestamp);
	} else if (slot == 9) {
		bb_interface_->set_bbox_9(values);
		bb_interface_->set_bbox_9_timestamp(timestamp);
	} else if (slot == 10) {
		bb_interface_->set_bbox_10(values);
		bb_interface_->set_bbox_10_timestamp(timestamp);
	} else if (slot == 11) {
		bb_interface_->set_bbox_11(values);
		bb_interface_->set_bbox_11_timestamp(timestamp);
	} else if (slot == 12) {
		bb_interface_->set_bbox_12(values);
		bb_interface_->set_bbox_12_timestamp(timestamp);
	} else if (slot == 13) {
		bb_interface_->set_bbox_13(values);
		bb_interface_->set_bbox_13_timestamp(timestamp);
	} else if (slot == 14) {
		bb_interface_->set_bbox_14(values);
		bb_interface_->set_bbox_14_timestamp(timestamp);
	}
	bb_interface_->write();
}

bool
PicamClientThread::bb_interface_message_received(Interface *interface, Message *message) noexcept
{
	bool status = false;
	if (message->is_of_type<PiCamPluginInterface::ActivateStreamMessage>()) {
		send_control_message(4);
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::ActivateMarkedStreamMessage>()) {
		send_control_message(5);
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::DeactivateStreamMessage>()) {
		send_control_message(6);
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::DeactivateMarkedStreamMessage>()) {
		send_control_message(7);
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::EnableWorkpieceDetectionMessage>()) {
		send_control_message(8);
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::EnableConveyorDetectionMessage>()) {
		send_control_message(9);
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::EnableSlideDetectionMessage>()) {
		send_control_message(10);
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::DisableDetectionMessage>()) {
		send_control_message(11);
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::SetConfidenceMessage>()) {
		PiCamPluginInterface::SetConfidenceMessage *msg =
		  (PiCamPluginInterface::SetConfidenceMessage *)message;
		send_control_message(12, msg->conf());
		status = true;
	} else if (message->is_of_type<PiCamPluginInterface::SetIOUMessage>()) {
		PiCamPluginInterface::SetIOUMessage *msg = (PiCamPluginInterface::SetIOUMessage *)message;
		send_control_message(13, msg->iou());
		status = true;
	}

	return status;
}

void
PicamClientThread::send_control_message(uint8_t message_type)
{
	uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
	                       std::chrono::system_clock::now().time_since_epoch())
	                       .count();

	char header[CONTROL_HEADER_SIZE];
	std::memcpy(&header[0], &message_type, 1);
	uint64_t network_timestamp = ntohll(timestamp);
	std::memcpy(&header[1], &network_timestamp, 8);

	send(sockfd_, header, CONTROL_HEADER_SIZE, 0);
}

void
PicamClientThread::send_control_message(uint8_t message_type, float payload)
{
	uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
	                       std::chrono::system_clock::now().time_since_epoch())
	                       .count();

	char header[CONTROL_HEADER_SIZE_PAYLOAD];
	std::memcpy(&header[0], &message_type, 1);
	uint64_t network_timestamp = ntohll(timestamp);
	std::memcpy(&header[1], &network_timestamp, 8);
	uint32_t network_payload = htonf(payload);
	std::memcpy(&header[9], &network_payload, 4);

	send(sockfd_, header, CONTROL_HEADER_SIZE, 0);
}

void
PicamClientThread::send_configure_message()
{
	uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
	                       std::chrono::system_clock::now().time_since_epoch())
	                       .count();

	char    header[CONFIGURE_MESSAGE_SIZE];
	uint8_t message_type = 14;
	std::memcpy(&header[0], &message_type, 1);
	uint64_t network_timestamp = ntohll(timestamp);
	std::memcpy(&header[1], &network_timestamp, 8);

	uint32_t network_rotation = htonl(config->get_int("plugins/picam_client/camera_matrix/rotation"));
	std::memcpy(&header[9], &network_rotation, 4);
	uint32_t network_old_ppx = htonf(config->get_float("plugins/picam_client/camera_matrix/old_ppx"));
	std::memcpy(&header[13], &network_old_ppx, 4);
	uint32_t network_old_ppy = htonf(config->get_float("plugins/picam_client/camera_matrix/old_ppy"));
	std::memcpy(&header[17], &network_old_ppy, 4);
	uint32_t network_old_f_y = htonf(config->get_float("plugins/picam_client/camera_matrix/old_f_y"));
	std::memcpy(&header[21], &network_old_f_y, 4);
	uint32_t network_old_f_x = htonf(config->get_float("plugins/picam_client/camera_matrix/old_f_x"));
	std::memcpy(&header[25], &network_old_f_x, 4);
	uint32_t network_new_ppx = htonf(config->get_float("plugins/picam_client/camera_matrix/new_ppx"));
	std::memcpy(&header[29], &network_new_ppx, 4);
	uint32_t network_new_ppy = htonf(config->get_float("plugins/picam_client/camera_matrix/new_ppy"));
	std::memcpy(&header[33], &network_new_ppy, 4);
	uint32_t network_new_f_y = htonf(config->get_float("plugins/picam_client/camera_matrix/new_f_y"));
	std::memcpy(&header[37], &network_new_f_y, 4);
	uint32_t network_new_f_x = htonf(config->get_float("plugins/picam_client/camera_matrix/new_f_x"));
	std::memcpy(&header[41], &network_new_f_x, 4);
	uint32_t network_k1 = htonf(config->get_float("plugins/picam_client/camera_matrix/k1"));
	std::memcpy(&header[45], &network_k1, 4);
	uint32_t network_k2 = htonf(config->get_float("plugins/picam_client/camera_matrix/k2"));
	std::memcpy(&header[49], &network_k2, 4);
	uint32_t network_k3 = htonf(config->get_float("plugins/picam_client/camera_matrix/k3"));
	std::memcpy(&header[53], &network_k3, 4);
	uint32_t network_k4 = htonf(config->get_float("plugins/picam_client/camera_matrix/k4"));
	std::memcpy(&header[57], &network_k4, 4);
	uint32_t network_k5 = htonf(config->get_float("plugins/picam_client/camera_matrix/k5"));
	std::memcpy(&header[61], &network_k5, 4);

	send(sockfd_, header, CONFIGURE_MESSAGE_SIZE, 0);
}

bool
PicamClientThread::receive_data(int sockfd, char *buffer, size_t size)
{
	size_t total_received = 0;
	while (total_received < size) {
		ssize_t received = read(sockfd, buffer + total_received, size - total_received);
		if (received <= 0) {
			return false;
		}
		total_received += received;
	}
	return true;
}

int
PicamClientThread::connect_to_server()
{
	if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		logger->log_error(name(), "Socket creation error");
		return -1;
	}

	server_addr_.sin_family = AF_INET;
	server_addr_.sin_port   = htons(server_port_);

	if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0) {
		logger->log_error(name(), "Invalid address or address not supported");
		close(sockfd_);
		return -1;
	}

	if (connect(sockfd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
		logger->log_error(name(), "Connection failed");
		close(sockfd_);
		return -1;
	}
	send_configure_message();
	send_control_message(13, config->get_float("plugins/picam_client/detection/iou"));
	send_control_message(12, config->get_float("plugins/picam_client/detection/conf"));

	return 0;
}

uint64_t
PicamClientThread::ntohll(uint64_t netlonglong)
{
	if (htonl(1) != 1) {
		return ((uint64_t)ntohl(netlonglong & 0xFFFFFFFF) << 32) | ntohl(netlonglong >> 32);
	} else {
		return netlonglong;
	}
}

float
PicamClientThread::ntohlf(float val)
{
	uint32_t temp;
	std::memcpy(&temp, &val, sizeof(temp));
	temp = ntohl(temp);
	float swapped;
	std::memcpy(&swapped, &temp, sizeof(swapped));
	return swapped;
}

uint32_t
PicamClientThread::htonf(float val)
{
	uint32_t temp;
	std::memcpy(&temp, &val, sizeof(temp));
	temp = htonl(temp);
	return temp;
}
