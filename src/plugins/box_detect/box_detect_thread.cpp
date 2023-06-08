/***************************************************************************
 *  box_detect_thread.cpp - box_detect
 *
 *  Plugin created: May 30 21:54:46 2023

 *  Copyright  2023 Daniel Honies
 *
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

#include "box_detect_thread.h"

using namespace fawkes;

using namespace std;

/**
 * @class BoxDetectThread "box_detect_thread.h"
 * Subscribes to TF data and publishes detected MPS in the box interface.
 * @author Daniel Honies	
*/

BoxDetectThread::BoxDetectThread()
: Thread("BoxDetectThread", Thread::OPMODE_CONTINUOUS),
  TransformAspect(TransformAspect::ONLY_LISTENER),
  BlackBoardInterfaceListener("BoxDetectThread")
{
}

void
BoxDetectThread::init()
{
	box_detect_ifs_ = new std::vector<fawkes::BoxInterface *>();
	for (int i = 1; i <= 15; i++) {
		std::string name = "box_detect_" + std::to_string(i);

		fawkes::BoxInterface *box_detect_if =
		  blackboard->open_for_writing<fawkes::BoxInterface>(name.c_str());
		box_detect_ifs_->push_back(box_detect_if);
	}

	tfifs_ = blackboard->open_multiple_for_reading<TransformInterface>("/tf/BOX*");
	std::list<TransformInterface *>::iterator i;
	for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
		bbil_add_data_interface(*i);
		bbil_add_reader_interface(*i);
		bbil_add_writer_interface(*i);
	}

	switch_if_ = blackboard->open_for_reading<SwitchInterface>("/switch/box_detect_enabled");
	bbil_add_data_interface(switch_if_);
	bbil_add_reader_interface(switch_if_);
	bbil_add_writer_interface(switch_if_);

	blackboard->register_listener(this);
	bbio_add_observed_create("TransformInterface", "/tf/BOX*");
	bbio_add_observed_create("SwitchInterface", "/switch/box_detect_enabled");
	blackboard->register_observer(this);

	client_ = node_handle->create_client<std_srvs::srv::SetBool>("/enable_box_detect");

}

void
BoxDetectThread::loop()
{
}

void
BoxDetectThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->unregister_observer(this);


	std::list<TransformInterface *>::iterator i;
	for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
		blackboard->close(*i);
	}
	tfifs_.clear();
}

void
BoxDetectThread::bb_interface_data_refreshed(fawkes::Interface *interface) throw()
{
	TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
	if (!tfif){
		SwitchInterface *swif = dynamic_cast<SwitchInterface *>(interface);
		if (!swif)
			return;
		swif->read();
		if(swif->is_enabled()){
			logger->log_warn(name(), "Box detect enabled");
			enabled_ = true;
			auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
			request->data = true;
			// Call the service
			auto future = client_->async_send_request(request);
			}
		else {
			logger->log_warn(name(), "Box detect disabled");
			enabled_ = false;
			auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
			request->data = false;
			// Call the service
			auto future = client_->async_send_request(request);
		}
		return;
	}
		

	tfif->read();
	string child_frame_id = tfif->child_frame();
	// log child_frame 
	logger->log_warn(name(), "Got update from transform interface: %s", child_frame_id.c_str());

	string box_type = child_frame_id.substr(4, 2);
	string zone     = child_frame_id.substr(7, 4);

	const char *b_type = box_type.c_str();
	const char *z = zone.c_str();
	int  rot   = stoi(child_frame_id.substr(12, child_frame_id.length() - 12));
	bool found = false;

	// go through all box_detect_ifs_ and check if the box is already in there
	for (int i = 0; i < box_detect_ifs_->size(); i++) {
		if (box_detect_ifs_->at(i)->box_type() == box_type && box_detect_ifs_->at(i)->zone()== zone) {
			logger->log_warn(name(), "Found box_detect_if at %d", i);
			found = true;
			box_detect_ifs_->at(i)->set_orientation(rot);
			box_detect_ifs_->at(i)->write();
		}
	}
	if (!found) {
		for (int i = 0; i < box_detect_ifs_->size(); i++) {
			if(strcmp(box_detect_ifs_->at(i)->box_type(), "") == 0)
			{
				logger->log_warn(name(), "Found empty box_detect_if at %d", i);
				found = true;
				box_detect_ifs_->at(i)->set_box_type(b_type);
				box_detect_ifs_->at(i)->set_zone(z);
				box_detect_ifs_->at(i)->set_orientation(rot);
				box_detect_ifs_->at(i)->write();
				break;
			}
		}
	}

	/*if (cfg_use_tf2_ && tfif->is_static_transform()) {
		publish_static_transforms_to_ros2();
	} else {
		tf2_msgs::msg::TFMessage tmsg;
		tmsg.transforms.push_back(create_transform_stamped(tfif));
		pub_tf_->publish(tmsg);
	}*/
}

void
BoxDetectThread::bb_interface_created(const char *type, const char *id) throw()
{
	if (strncmp(type, "TransformInterface", INTERFACE_TYPE_SIZE_) != 0)
		return;

	TransformInterface *tfif;
	try {
		//logger->log_info(name(), "Opening %s:%s", type, id);
		tfif = blackboard->open_for_reading<TransformInterface>(id);
	} catch (Exception &e) {
		// ignored
		logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
		return;
	}

	try {
		bbil_add_data_interface(tfif);
		bbil_add_reader_interface(tfif);
		bbil_add_writer_interface(tfif);
		blackboard->update_listener(this);
		tfifs_.push_back(tfif);
	} catch (Exception &e) {
		blackboard->close(tfif);
		logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
		return;
	}
}

void
BoxDetectThread::bb_interface_writer_removed(fawkes::Interface *interface,
                                             fawkes::Uuid       instance_serial) noexcept
{
	conditional_close(interface);
}

void
BoxDetectThread::bb_interface_reader_removed(fawkes::Interface *interface,
                                             fawkes::Uuid       instance_serial) noexcept
{
	conditional_close(interface);
}

void
BoxDetectThread::conditional_close(Interface *interface) noexcept
{
	// Verify it's a TransformInterface
	TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
	if (!tfif)
		return;

	std::list<TransformInterface *>::iterator i;
	for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
		if (*interface == **i) {
			if (!interface->has_writer() && (interface->num_readers() == 1)) {
				// It's only us
				logger->log_info(name(), "Last on %s, closing", interface->uid());
				bbil_remove_data_interface(*i);
				bbil_remove_reader_interface(*i);
				bbil_remove_writer_interface(*i);
				blackboard->update_listener(this);
				blackboard->close(*i);
				tfifs_.erase(i);
				break;
			}
		}
	}
}
