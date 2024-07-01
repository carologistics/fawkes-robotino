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

#include <cstdint>
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

using namespace fawkes;
using namespace cv;
using namespace dnn;

#define HEADER_SIZE_1 16 // Header size for message of type 1
#define HEADER_SIZE_2 16 // Header size for message of type 2
#define HEADER_SIZE_3 28 // Header size for message of type 3

/** @class PicamClientThread "picam_client_thread.h"
 * @author Daniel Swoboda
 */

/** Constructor. */
PicamClientThread::PicamClientThread() : Thread("PicamClientThread", Thread::OPMODE_CONTINUOUS)
{
}

void
PicamClientThread::init()
{
	logger->log_info(name(), "Initializing Picam Client");
	std::cout << "OpenCV version : " << CV_VERSION << std::endl;

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

	// Create socket and connect
	if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		std::cerr << "Socket creation error" << std::endl;
		return;
	}

	server_addr_.sin_family = AF_INET;
	server_addr_.sin_port   = htons(server_port_);

	if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0) {
		std::cerr << "Invalid address/ Address not supported" << std::endl;
		return;
	}

	if (connect(sockfd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
		std::cerr << "Connection Failed" << std::endl;
		return;
	}
}

void
PicamClientThread::loop()
{
	data_.clear();

	// read the message type
	data_.resize(1);
	if (!receive_data(sockfd_, data_.data(), 1)) {
		std::cerr << "Failed to read header" << std::endl;
		return;
	}
	uint8_t message_type;
	std::memcpy(&message_type, &data_[0], 1);

	if (message_type == 1 || message_type == 2) {
		data_.clear();

		// read header for type 1 and 2 messages
		data_.resize(HEADER_SIZE_1);
		if (!receive_data(sockfd_, data_.data(), HEADER_SIZE_1)) {
			std::cerr << "Failed to read header" << std::endl;
			return;
		}
		uint32_t timestamp, width, height, length;

		std::memcpy(&timestamp, &data_[0], 4);
		std::memcpy(&height, &data_[4], 4);
		std::memcpy(&width, &data_[8], 4);
		std::memcpy(&length, &data_[12], 4);

		timestamp = ntohl(timestamp);
		height    = ntohl(height);
		width     = ntohl(width);
		length    = ntohl(length);

		// read the frame payload based on the length
		std::vector<char> image_data(length);
		if (!receive_data(sockfd_, image_data.data(), length)) {
			std::cerr << "Failed to read image data" << std::endl;
			return;
		}

		// decode base64
		std::string base64_image(image_data.begin(), image_data.end());
		std::string decoded_image = base64_decode(base64_image);

		// decode image using OpenCV
		std::vector<uchar> img_data(decoded_image.begin(), decoded_image.end());
		cv::Mat            img = cv::imdecode(img_data, cv::IMREAD_COLOR);

		if (img.empty()) {
			std::cerr << "Failed to decode image" << std::endl;
			return;
		}

		// write received image in the right shared memory buffer, based on type
		if (message_type == 1) {
			firevision::convert(firevision::BGR,
			                    firevision::BGR,
			                    img.data,
			                    shm_buffer_->buffer(),
			                    camera_width_,
			                    camera_height_);
		}
		if (message_type == 2) {
			firevision::convert(firevision::BGR,
			                    firevision::BGR,
			                    img.data,
			                    shm_buffer_res_->buffer(),
			                    camera_width_,
			                    camera_height_);
		}

	} else if (message_type == 3) {
		data_.clear();
		// read the message header of type 3
		data_.resize(HEADER_SIZE_3);
		if (!receive_data(sockfd_, data_.data(), HEADER_SIZE_3)) {
			std::cerr << "Failed to read header" << std::endl;
			return;
		}

		// unpack the data
		uint32_t timestamp, x, y, h, w, cls;
		float    acc;
		std::memcpy(&timestamp, &data_[0], 4);
		std::memcpy(&x, &data_[4], 4);
		std::memcpy(&y, &data_[8], 4);
		std::memcpy(&h, &data_[12], 4);
		std::memcpy(&w, &data_[16], 4);
		std::memcpy(&acc, &data_[20], 4);
		std::memcpy(&cls, &data_[24], 4);

		// convert to host byte order
		timestamp = ntohl(timestamp);
		x         = ntohl(x);
		y         = ntohl(y);
		h         = ntohl(h);
		w         = ntohl(w);
		acc       = ntohl(acc);
		cls       = ntohl(cls);
	}
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
