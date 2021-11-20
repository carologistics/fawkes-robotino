
/***************************************************************************
 *  visualisation.hpp -
 *
 *  Created: Thr 13. April 16:44:00 CEST 2016
 *  Copyright  2016 Tobias Neumann
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

#ifndef _VISUALISATION_
#define _VISUALISATION_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <libs/tf/types.h>
#include <core/utils/lockptr.h>

#include <string>

/**
 * @class Visualisation
 * @brief Publish various markers as ROS messages to aid with visual debuggin in rviz
 */
class Visualisation
{
private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  pub_markers_;

  visualization_msgs::msg::Marker draw_normal(pcl::PCLHeader header, fawkes::tf::Vector3 centroid, fawkes::tf::Quaternion rotation)
  {
    visualization_msgs::msg::Marker arrow;
    arrow.header = pcl_conversions::fromPCL( header );
    arrow.ns = "conveyor_normal";
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.005;
    arrow.scale.z = 0.005;
    arrow.color.g = 1.0f;
    arrow.color.a = 1.0;
    geometry_msgs::msg::Point start, end;
    start.x = centroid.x();
    start.y = centroid.y();
    start.z = centroid.z();
    arrow.points.push_back(start);
    fawkes::tf::Vector3 normal(0, 0, 1);
    normal = normal.rotate(fawkes::tf::Vector3(rotation.x(), rotation.y(), rotation.z()), rotation.w());
    end.x = centroid.x() - normal.x() * 0.05;
    end.y = centroid.y() - normal.y() * 0.05;
    end.z = centroid.z() - normal.z() * 0.05;
    arrow.points.push_back(end);

    return arrow;
  }

  visualization_msgs::msg::Marker draw_plane(pcl::PCLHeader header, fawkes::tf::Vector3 centroid, fawkes::tf::Quaternion rotation)
  {
    visualization_msgs::msg::Marker plane;

    plane.header = pcl_conversions::fromPCL(header);
    plane.ns = "conveyor_plane";
    plane.action = visualization_msgs::msg::Marker::ADD;
    plane.id = 0;
    plane.scale.x = 1.;
    plane.scale.y = 1.;
    plane.scale.z = 1.;
    plane.color.r = 1.0f;
    plane.color.a = 1.0;
    plane.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    geometry_msgs::msg::Point tl, tr, bl, br;

    // get plane version 2: conveyor size from centroid
    float t, b, l, r, dl, dr;
    const float width = 0.019;
    t = centroid.y() - 0.015;
    b = centroid.y() + 0.023;
    float plane_angle = 0;//std::atan2( coeff->values[0], - coeff->values[2] );
    r = centroid.x() + std::cos( plane_angle ) * width;
    l = centroid.x() - std::cos( plane_angle ) * width;
    dr = centroid.z() + std::sin( plane_angle ) * width;
    dl = centroid.z() - std::sin( plane_angle ) * width;

    tl.x = l;
    tl.y = t;
    tl.z = dl;
    tr.x = r;
    tr.y = t;
    tr.z = dr;
    bl.x = l;
    bl.y = b;
    bl.z = dl;
    br.x = r;
    br.y = b;
    br.z = dr;


    // end get plane
    plane.points.push_back(tl);
    plane.points.push_back(bl);
    plane.points.push_back(tr);
    plane.points.push_back(tr);
    plane.points.push_back(br);
    plane.points.push_back(bl);
    plane.colors.push_back(plane.color);
    plane.colors.push_back(plane.color);
    plane.colors.push_back(plane.color);
    plane.colors.push_back(plane.color);
    plane.colors.push_back(plane.color);
    plane.colors.push_back(plane.color);

    return plane;
  }

public:

  /**
   * @brief marker_draw generate a marker and publish it to ROS
   * @param header header of the source cloud, used as header for the marker
   * @param centroid the plane's centnroid
   * @param rotation the plane's rotation
   */
  void marker_draw(pcl::PCLHeader header, fawkes::tf::Vector3 centroid, fawkes::tf::Quaternion rotation)
  {
    // just viz stuff from here
    visualization_msgs::msg::MarkerArray ma;
    // add arrow normal
    visualization_msgs::msg::Marker arrow = draw_normal(header, centroid, rotation);
    ma.markers.push_back(arrow);
    // add plane
//    visualization_msgs::msg::Marker plane = draw_plane(header, centroid, rotation);
//    ma.markers.push_back(plane);
    pub_markers_->publish(ma);
  }

  /**
   * @brief Visualisation constructor
   * @param rosnode Node that should advertise the visualization marker array
   */
  Visualisation(rclcpp::Node::SharedPtr node_handle)
  {
    pub_markers_ = node_handle->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1);
  }
};


#endif
