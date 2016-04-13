
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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <string>

class Visualisation
{
private:
  ros::Publisher  pub_markers_;

  visualization_msgs::Marker draw_normal(pcl::PCLHeader header, Eigen::Vector4f centroid, pcl::ModelCoefficients::Ptr coeff)
  {
    visualization_msgs::Marker arrow;
    arrow.header = pcl_conversions::fromPCL( header );
    arrow.ns = "conveyor_normal";
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.005;
    arrow.scale.z = 0.005;
    arrow.color.g = 1.0f;
    arrow.color.a = 1.0;
    geometry_msgs::Point start, end;
    start.x = centroid(0);
    start.y = centroid(1);
    start.z = centroid(2);
    arrow.points.push_back(start);
    end.x = centroid(0) - coeff->values[0] * 0.05;
    end.y = centroid(1) - coeff->values[1] * 0.05;
    end.z = centroid(2) - coeff->values[2] * 0.05;
    arrow.points.push_back(end);

    return arrow;
  }

  visualization_msgs::Marker draw_plane(pcl::PCLHeader header, Eigen::Vector4f centroid, pcl::ModelCoefficients::Ptr coeff)
  {
    visualization_msgs::Marker plane;

    plane.header = pcl_conversions::fromPCL(header);
    plane.ns = "conveyor_plane";
    plane.action = visualization_msgs::Marker::ADD;
    plane.id = 0;
    plane.scale.x = 1.;
    plane.scale.y = 1.;
    plane.scale.z = 1.;
    plane.color.r = 1.0f;
    plane.color.a = 1.0;
    plane.type = visualization_msgs::Marker::TRIANGLE_LIST;
    geometry_msgs::Point tl, tr, bl, br;

    // get plane version 2: conveyor size from centroid
    float t, b, l, r, dl, dr;
    const float width = 0.019;
    t = centroid(1) - 0.015;
    b = centroid(1) + 0.023;
    float plane_angle = std::atan2( coeff->values[0], - coeff->values[2] );
    r = centroid(0) + std::cos( plane_angle ) * width;
    l = centroid(0) - std::cos( plane_angle ) * width;
    dr = centroid(2) + std::sin( plane_angle ) * width;
    dl = centroid(2) - std::sin( plane_angle ) * width;

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

  void marker_draw(pcl::PCLHeader header, Eigen::Vector4f centroid, pcl::ModelCoefficients::Ptr coeff)
  {
    // just viz stuff from here
    visualization_msgs::MarkerArray ma;
    // add arrow normal
    visualization_msgs::Marker arrow = draw_normal(header, centroid, coeff);
    ma.markers.push_back(arrow);
    // add plane
    visualization_msgs::Marker plane = draw_plane(header, centroid, coeff);
    ma.markers.push_back(plane);
    pub_markers_.publish(ma);
  }

  Visualisation(fawkes::LockPtr<ros::NodeHandle> rosnode)
  {
    pub_markers_ = rosnode->advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
  }
};


#endif
