
/***************************************************************************
 *  mps-laser-gen_thread.cpp - mps-laser-gen
 *
 *  Plugin created: Thu Jun 30 21:54:46 2016

 *  Copyright  2016  Tim Niemueller
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

#include "mps-laser-gen_thread.h"

#include <navgraph/navgraph.h>
#include <ros/ros.h>
#include <tf/types.h>
#include <utils/math/angle.h>
#include <utils/math/lines.h>
#include <utils/time/time.h>
#include <visualization_msgs/MarkerArray.h>

using namespace fawkes;

/** @class MPSLaserGenThread "mps-laser-gen_thread.h"
 * Generate laser data from known MPS.
 * @author Tim Niemueller
 */

MPSLaserGenThread::MPSLaserGenThread()
: Thread("MPSLaserGenThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
  TransformAspect(TransformAspect::ONLY_LISTENER)
{
}

void
MPSLaserGenThread::init()
{
	vispub_ = rosnode->advertise<visualization_msgs::MarkerArray>("visualization_marker_array",
	                                                              100,
	                                                              /* latching */ true);

	laser_if_            = blackboard->open_for_writing<Laser360Interface>("Laser MPS");
	laser_box_filter_if_ = blackboard->open_for_reading<LaserBoxFilterInterface>("Laser Box Filter");
	load_config();
}

void
MPSLaserGenThread::loop()
{
	navgraph.lock();
	std::vector<fawkes::NavGraphNode> nodes = navgraph->nodes();
	navgraph.unlock();

	const float mps_length = cfg_mps_length_;
	const float mps_width  = cfg_mps_width_;

	const float mps_length_2 = mps_length / 2.;
	const float mps_width_2  = mps_width / 2.;

	std::string global_frame = "map";
	std::string sensor_frame = "base_link";

	float data[360];
	for (unsigned int i = 0; i < 360; ++i)
		data[i] = std::numeric_limits<float>::quiet_NaN();

	fawkes::Time         source_time(0, 0);
	tf::StampedTransform tf_transform;
	try {
		tf_listener->lookup_transform(sensor_frame, global_frame, source_time, tf_transform);
	} catch (Exception &e) {
		laser_if_->set_frame(sensor_frame.c_str());
		laser_if_->set_distances(data);
		laser_if_->write();
		logger->log_warn(name(), "Failed to acquire transform for sensor frame, skipping loop");
		return;
	}

	Eigen::Rotation2Df   sensor_rotation(tf::get_yaw(tf_transform.getRotation()));
	Eigen::Translation2f sensor_translation(tf_transform.getOrigin().x(),
	                                        tf_transform.getOrigin().y());

	Eigen::Transform<float, 2, Eigen::Affine> transform = sensor_translation * sensor_rotation;

	visualization_msgs::MarkerArray m;
	// #if ROS_VERSION_MINIMUM(1,10,0)
	//   {
	// 	  visualization_msgs::Marker delop;
	// 	  delop.header.frame_id = "map";
	// 	  delop.header.stamp = ros::Time::now();
	// 	  delop.ns = "mps-laser-gen";
	// 	  delop.id = 0;
	// 	  delop.action = 3; // visualization_msgs::Marker::DELETEALL;
	// 	  m.markers.push_back(delop);
	//   }
	// #endif
	size_t id_num = 0;

	for (const fawkes::NavGraphNode &n : nodes) {
		if (n.has_property("mps") && n.property_as_bool("mps")) {
			float ori = 0.;
			if (n.has_property("orientation")) {
				ori = n.property_as_float("orientation");
			}

			MPS mps;
			// send a message to the box_filter laser filter if needed
			if (cfg_enable_mps_box_filter_ == true && mpses.count(n.name()) == 0) {
				mps.center = Eigen::Vector2f(n.x(), n.y());

				mps.corners[0] = Eigen::Vector2f(mps_width_2, -mps_length_2);
				mps.corners[1] = Eigen::Vector2f(-mps_width_2, -mps_length_2);
				mps.corners[2] = Eigen::Vector2f(-mps_width_2, mps_length_2);
				mps.corners[3] = Eigen::Vector2f(mps_width_2, mps_length_2);

				Eigen::Rotation2Df rot(ori);
				mps.corners[0] = (rot * mps.corners[0]) + mps.center;
				mps.corners[1] = (rot * mps.corners[1]) + mps.center;
				mps.corners[2] = (rot * mps.corners[2]) + mps.center;
				mps.corners[3] = (rot * mps.corners[3]) + mps.center;

				LaserBoxFilterInterface::CreateNewBoxFilterMessage *box_filter_msg =
				  new LaserBoxFilterInterface::CreateNewBoxFilterMessage();
				box_filter_msg->set_p1(0, mps.corners[0][0]);
				box_filter_msg->set_p1(1, mps.corners[0][1]);
				box_filter_msg->set_p2(0, mps.corners[1][0]);
				box_filter_msg->set_p2(1, mps.corners[1][1]);
				box_filter_msg->set_p3(0, mps.corners[2][0]);
				box_filter_msg->set_p3(1, mps.corners[2][1]);
				box_filter_msg->set_p4(0, mps.corners[3][0]);
				box_filter_msg->set_p4(1, mps.corners[3][1]);

				laser_box_filter_if_->read();
				laser_box_filter_if_->msgq_enqueue(box_filter_msg);

				mpses[n.name()] = mps;
			}

			if (cfg_enable_mps_laser_gen_ == true) {
				mps.center = transform * Eigen::Vector2f(n.x(), n.y());

				mps.corners[0] = sensor_rotation * Eigen::Vector2f(mps_width_2, -mps_length_2);
				mps.corners[1] = sensor_rotation * Eigen::Vector2f(-mps_width_2, -mps_length_2);
				mps.corners[2] = sensor_rotation * Eigen::Vector2f(-mps_width_2, mps_length_2);
				mps.corners[3] = sensor_rotation * Eigen::Vector2f(mps_width_2, mps_length_2);

				Eigen::Rotation2Df rot(ori);
				mps.corners[0] = (rot * mps.corners[0]) + mps.center;
				mps.corners[1] = (rot * mps.corners[1]) + mps.center;
				mps.corners[2] = (rot * mps.corners[2]) + mps.center;
				mps.corners[3] = (rot * mps.corners[3]) + mps.center;

				float dists[4]  = {mps.corners[0].norm(),
                          mps.corners[1].norm(),
                          mps.corners[2].norm(),
                          mps.corners[3].norm()};
				mps.closest_idx = 0;
				for (unsigned int i = 1; i < 4; ++i) {
					if (dists[i] < dists[mps.closest_idx])
						mps.closest_idx = i;
				}

				mps.adjacent_1 = (mps.closest_idx == 0) ? 3 : mps.closest_idx - 1;
				mps.adjacent_2 = (mps.closest_idx == 3) ? 0 : mps.closest_idx + 1;

				mps.bearing = atan2f(mps.corners[mps.closest_idx][1], mps.corners[mps.closest_idx][0]);
				// logger->log_info(name(), "Station %s bearing %f", n.name().c_str(),
				// mps.bearing);

				mpses[n.name()] = mps;

				{
					visualization_msgs::Marker sphere;
					sphere.header.frame_id    = sensor_frame;
					sphere.header.stamp       = ros::Time::now();
					sphere.ns                 = "mps-laser-gen";
					sphere.id                 = id_num++;
					sphere.type               = visualization_msgs::Marker::SPHERE;
					sphere.action             = visualization_msgs::Marker::ADD;
					sphere.pose.position.x    = mps.center[0];
					sphere.pose.position.y    = mps.center[1];
					sphere.pose.position.z    = 0.;
					sphere.pose.orientation.w = 1.;
					sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.1;
					sphere.color.r                                   = 0.f;
					sphere.color.g                                   = 1.f;
					sphere.color.b                                   = 0.f;
					sphere.color.a                                   = 1.0;
					sphere.lifetime                                  = ros::Duration(0, 0);
					m.markers.push_back(sphere);
				}

				for (unsigned int i = 0; i < 4; ++i) {
					visualization_msgs::Marker sphere;
					sphere.header.frame_id    = sensor_frame;
					sphere.header.stamp       = ros::Time::now();
					sphere.ns                 = "mps-laser-gen";
					sphere.id                 = id_num++;
					sphere.type               = visualization_msgs::Marker::SPHERE;
					sphere.action             = visualization_msgs::Marker::ADD;
					sphere.pose.position.x    = mps.corners[i][0];
					sphere.pose.position.y    = mps.corners[i][1];
					sphere.pose.position.z    = 0.;
					sphere.pose.orientation.w = 1.;
					sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05;
					if (i == mps.closest_idx) {
						sphere.color.r = 0.f;
						sphere.color.b = 1.f;
					} else if (i == mps.adjacent_1 || i == mps.adjacent_2) {
						sphere.color.r = 1.f;
						sphere.color.b = 1.f;
					} else {
						sphere.color.r = 1.f;
						sphere.color.b = 0.f;
					}
					sphere.color.g  = 0.f;
					sphere.color.a  = 1.0;
					sphere.lifetime = ros::Duration(0, 0);
					m.markers.push_back(sphere);
				}
			}
		}
	}

	for (unsigned int i = 0; i < 360; ++i) {
		float a = normalize_mirror_rad(deg2rad(i));

		for (const auto &mps : mpses) {
			if ((a - mps.second.bearing) < 0.3) {
				// Consider
				Eigen::Vector2f beam(20., 0);
				beam = Eigen::Rotation2Df(a) * beam;
				Eigen::Vector2f intersect_1 =
				  line_segm_intersection(mps.second.corners[mps.second.closest_idx],
				                         mps.second.corners[mps.second.adjacent_1],
				                         Eigen::Vector2f(0, 0),
				                         beam);
				if (intersect_1.allFinite()) {
					float l = intersect_1.norm();
					if (std::isnan(data[i]) || l < data[i])
						data[i] = l;
				}

				Eigen::Vector2f intersect_2 =
				  line_segm_intersection(mps.second.corners[mps.second.closest_idx],
				                         mps.second.corners[mps.second.adjacent_2],
				                         Eigen::Vector2f(0, 0),
				                         beam);
				if (intersect_2.allFinite()) {
					float l = intersect_2.norm();
					if (std::isnan(data[i]) || l < data[i])
						data[i] = l;
				}
			}
		}
	}
	laser_if_->set_frame(sensor_frame.c_str());
	laser_if_->set_distances(data);
	laser_if_->write();

	vispub_.publish(m);
}

void
MPSLaserGenThread::finalize()
{
	float data[360];
	for (unsigned int i = 0; i < 360; ++i)
		data[i] = std::numeric_limits<float>::quiet_NaN();
	laser_if_->set_frame("");
	laser_if_->set_distances(data);
	laser_if_->write();

	vispub_.shutdown();
	blackboard->close(laser_if_);
}

void
MPSLaserGenThread::load_config()
{
	cfg_enable_mps_laser_gen_  = config->get_bool((CFG_PREFIX "enable_laser_gen"));
	cfg_enable_mps_box_filter_ = config->get_bool((CFG_PREFIX "enable_mps_box_filter"));
	cfg_mps_length_            = config->get_float((CFG_PREFIX "mps_length"));
	cfg_mps_width_             = config->get_float((CFG_PREFIX "mps_width"));
}
