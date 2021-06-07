/***************************************************************************
 *  plugin_template_thread.cpp - template thread
 *
 *  Created: Mi 23. Mai 17:44:14 CEST 2012
 *  Copyright  2012 Daniel Ewert, 2014 Florian Nolden
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

#include "marker_writer_thread.h"

#include <interfaces/Position3DInterface.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>

#define CFG_PREFIX "/plugins/markerwriter/"

using namespace fawkes;
using namespace std;

/** @class MarkerWriterThread "marker_writer_thread.h"
 * Read all Position3dInterfaces and make the data accessible via ROS/rviz
 * @author Daniel Ewert
 */

/** Constructor. */
MarkerWriterThread::MarkerWriterThread()
: Thread("MarkerWriterThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
	pub_ = new ros::Publisher();
}

void
MarkerWriterThread::init()
{
	cfg_duration_           = config->get_float(CFG_PREFIX "display_duration");
	cfg_ignore_visibillity_ = config->get_bool(CFG_PREFIX "ignore_visibility_history");

	*pub_ =
	  rosnode->advertise<visualization_msgs::MarkerArray>("Position3dInterface_marker_array", 100);

	string wildcard_pattern = config->get_string(CFG_PREFIX "wildcard_pattern");
	pos_ifs_ = blackboard->open_multiple_for_reading<Position3DInterface>(wildcard_pattern.c_str());
	for (Position3DInterface *pos_if : pos_ifs_) {
		logger->log_debug(name(), "visualizing %s", pos_if->id());
	}
}

bool
MarkerWriterThread::prepare_finalize_user()
{
	return true;
}

void
MarkerWriterThread::finalize()
{
	for (Position3DInterface *pos_if : pos_ifs_) {
		blackboard->close(pos_if);
	}
	pub_->shutdown();
	delete pub_;
}

void
MarkerWriterThread::loop()
{
	unsigned int                    idnum = 0;
	visualization_msgs::MarkerArray m;
	for (Position3DInterface *pos_if : pos_ifs_) {
		pos_if->read();

		if (cfg_ignore_visibillity_ || pos_if->visibility_history() > 0) {
			visualization_msgs::Marker text;
			text.header.frame_id    = "/base_link";
			text.header.stamp       = ros::Time::now();
			text.ns                 = "robotino";
			text.id                 = idnum++;
			text.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
			text.action             = visualization_msgs::Marker::ADD;
			text.pose.position.x    = pos_if->translation(0);
			text.pose.position.y    = pos_if->translation(1);
			text.pose.position.z    = pos_if->translation(2) + 0.13;
			text.pose.orientation.w = 1.;
			text.scale.z            = 0.05; // 5cm high
			text.color.r = text.color.g = text.color.b = 1.0f;
			text.color.a                               = 1.0;
			text.lifetime                              = ros::Duration(cfg_duration_, 0);
			text.text                                  = pos_if->id();
			m.markers.push_back(text);

			visualization_msgs::Marker sphere;
			sphere.header.frame_id    = "/base_link";
			sphere.header.stamp       = ros::Time::now();
			sphere.ns                 = "robotino";
			sphere.id                 = idnum++;
			sphere.type               = visualization_msgs::Marker::CYLINDER;
			sphere.action             = visualization_msgs::Marker::ADD;
			sphere.pose.position.x    = pos_if->translation(0);
			sphere.pose.position.y    = pos_if->translation(1);
			sphere.pose.position.z    = pos_if->translation(2);
			sphere.pose.orientation.w = 1.;
			sphere.scale.x = sphere.scale.y = 0.08;
			sphere.scale.z                  = 0.09;
			sphere.color.r                  = 1.f;
			sphere.color.g                  = 0.f;
			sphere.color.b                  = 0.f;
			sphere.color.a                  = 1.0;
			sphere.lifetime                 = ros::Duration(cfg_duration_, 0);
			m.markers.push_back(sphere);

			if (pos_if->rotation(0) != 0 || pos_if->rotation(1) != 0 || pos_if->rotation(2) != 0) {
				// if object has a orientation visualize it
				visualization_msgs::Marker arrow;
				arrow.header.frame_id    = "/base_link";
				arrow.header.stamp       = ros::Time::now();
				arrow.ns                 = "robotino";
				arrow.id                 = idnum++;
				arrow.type               = visualization_msgs::Marker::ARROW;
				arrow.action             = visualization_msgs::Marker::ADD;
				arrow.pose.position.x    = pos_if->translation(0);
				arrow.pose.position.y    = pos_if->translation(1);
				arrow.pose.position.z    = pos_if->translation(2);
				arrow.pose.orientation.x = pos_if->rotation(0);
				arrow.pose.orientation.y = pos_if->rotation(1);
				arrow.pose.orientation.z = pos_if->rotation(2);
				arrow.pose.orientation.w = 1.;
				arrow.scale.x = sphere.scale.y = 0.08;
				arrow.scale.z                  = 0.09;
				arrow.color.r                  = 1.f;
				arrow.color.g                  = 1.f;
				arrow.color.b                  = 0.f;
				arrow.color.a                  = 1.0;
				arrow.lifetime                 = ros::Duration(cfg_duration_, 0);

				m.markers.push_back(arrow);
			}
		}
		pub_->publish(m);
	}
}
