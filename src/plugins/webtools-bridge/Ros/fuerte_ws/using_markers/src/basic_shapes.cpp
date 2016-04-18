/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// %EndTag(INCLUDES)%

// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub1 = n.advertise<visualization_msgs::Marker>("visualization_marker_R1", 1);
  ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker_R2", 1);
  ros::Publisher marker_pub3 = n.advertise<visualization_msgs::Marker>("visualization_marker_R3", 1);
// %EndTag(INIT)%

  // Set our initial shape type to be a ARROQ
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::ARROW;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
  while (ros::ok())
  {
    visualization_msgs::Marker marker1;
    visualization_msgs::Marker marker2;
    visualization_msgs::Marker marker3;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker1.header.frame_id = "/base_link";
    marker1.header.stamp = ros::Time::now();
    marker2.header.frame_id = "/base_link";
    marker2.header.stamp = ros::Time::now();
    marker3.header.frame_id = "/base_link";
    marker3.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker1.ns = "basic_shapes1";
    marker1.id = 0;
    marker2.ns = "basic_shapes1";
    marker2.id = 1;
    marker3.ns = "basic_shapes1";
    marker3.id = 2;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker1.type = shape;
    marker2.type = shape;
    marker3.type = shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
// %Tag(ACTION)%
    marker1.action = visualization_msgs::Marker::ADD;
    marker2.action = visualization_msgs::Marker::ADD;
    marker3.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker1.pose.position.x = 0;
    marker1.pose.position.y = 0;
    marker1.pose.position.z = 0;
    marker1.pose.orientation.x = 0.0;
    marker1.pose.orientation.y = 0.0;
    marker1.pose.orientation.z = 0.0;
    marker1.pose.orientation.w = 1.0;
     marker1.pose.position.x = 0;
    marker2.pose.position.y = 0;
    marker2.pose.position.z = 0;
    marker2.pose.orientation.x = 0.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 1.0;
     marker3.pose.position.x = 0;
    marker3.pose.position.y = 0;
    marker3.pose.position.z = 0;
    marker3.pose.orientation.x = 0.0;
    marker3.pose.orientation.y = 0.0;
    marker3.pose.orientation.z = 0.0;
    marker3.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker1.scale.x = 0.35;
    marker1.scale.y = 0.35;
    marker1.scale.z = 0.35;
    
    marker2.scale.x = 0.35;
    marker2.scale.y = 0.35;
    marker2.scale.z = 0.35;
    
    marker3.scale.x = 0.35;
    marker3.scale.y = 0.35;
    marker3.scale.z = 0.35;
    

// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker1.color.r = 1.0f;
    marker1.color.g = 0.0f;
    marker1.color.b = 0.0f;
    marker1.color.a = 1.0;

     marker2.color.r = 0.0f;
    marker2.color.g = 1.0f;
    marker2.color.b = 0.0f;
    marker2.color.a = 1.0;

     marker3.color.r = 0.0f;
    marker3.color.g = 0.0f;
    marker3.color.b = 1.0f;
    marker3.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker1.lifetime = ros::Duration();
    marker2.lifetime = ros::Duration();
    marker3.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
    while (marker_pub1.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub1.publish(marker1);

      while (marker_pub2.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub2.publish(marker2);

      while (marker_pub3.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub3.publish(marker3);
// %EndTag(PUBLISH)%

    // Cycle between different shapes
// %Tag(CYCLE_SHAPES)%
  /*  switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }*/
// %EndTag(CYCLE_SHAPES)%

// %Tag(SLEEP_END)%
    r.sleep();
  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
