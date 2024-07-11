/***************************************************************************
 *  picam_client_thread.h   - Thread to subscribe to camera streams from the
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

#ifndef __PLUGINS_PICAM_CLIENT_THREAD_H_
#define __PLUGINS_PICAM_CLIENT_THREAD_H_

#include <arpa/inet.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/PiCamPluginInterface.h>
#include <utils/time/time.h>

#include <cmath>
#include <opencv2/dnn.hpp>
#include <string.h>
#include <unistd.h>

// firevision camera
#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>

namespace fawkes {
class ObjectTrackingInterface;
} // namespace fawkes

namespace firevision {
class SharedMemoryImageBuffer;
} // namespace firevision

class PicamClientThread : public fawkes::Thread,
                          public fawkes::ClockAspect,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::BlackBoardAspect,
                          public fawkes::BlackBoardInterfaceListener
{
public:
	PicamClientThread();

	virtual void init();
	virtual void loop();

private:
	//server params
	std::string server_ip_;
	int         server_port_;

	//camera params
	int camera_width_;
	int camera_height_;

	//blackboard interface
	fawkes::PiCamPluginInterface *bb_interface_;

	//connection vars
	bool connected_          = false;
	int  disconnect_counter_ = 0;

	//socket vars
	int                sockfd_;
	struct sockaddr_in server_addr_;
	char               buffer_[1024];
	std::vector<char>  data_;

	//shared memory buffer
	std::string                          shm_id_;
	firevision::SharedMemoryImageBuffer *shm_buffer_;
	std::string                          shm_id_res_;
	firevision::SharedMemoryImageBuffer *shm_buffer_res_;
	bool                                 shm_active_;

	//message vars
	uint64_t last_msg_time_;
	int      msg_counter_;

	//functions
	void reset_interface();
	void write_to_interface(int      slot,
	                        float    x,
	                        float    y,
	                        float    h,
	                        float    w,
	                        float    acc,
	                        float    cl,
	                        uint64_t timestamp_secs,
	                        uint64_t timestamp_usecs);

	bool     receive_data(int sockfd, char *buffer, size_t size);
	bool     bb_interface_message_received(fawkes::Interface *interface,
	                                       fawkes::Message   *message) throw();
	void     send_control_message(uint8_t message_type);
	void     send_control_message(uint8_t message_type, float payload);
	void     send_configure_message();
	int      connect_to_server();
	uint64_t ntohll(uint64_t val);
	float    ntohlf(float val);
};
#endif
