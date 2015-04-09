
/***************************************************************************
 *  navgraph_generator_mps_thread.cpp - Generate navgraph for MPS
 *
 *  Created: Tue Jan 13 15:33:37 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_generator_mps_thread.h"

#include <navgraph/yaml_navgraph.h>
#include <core/threading/mutex_locker.h>
#include <navgraph/navgraph.h>
#include <interfaces/NavGraphGeneratorInterface.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>
#include <blackboard/utils/on_message_waker.h>
#include <utils/math/angle.h>
#include <utils/math/eigen.h>
#include <limits>
#include <memory>

using namespace fawkes;

/** @class NavGraphGeneratorMPSThread "navgraph_generator_mps_thread.h"
 * Block navgraph paths based on laser clusters.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphGeneratorMPSThread::NavGraphGeneratorMPSThread()
  : Thread("NavGraphGeneratorMPSThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
NavGraphGeneratorMPSThread::~NavGraphGeneratorMPSThread()
{
}

void
NavGraphGeneratorMPSThread::init()
{
  cfg_global_frame_      = config->get_string("/frames/fixed");
  cfg_mps_width_         = config->get_float("/navgraph-generator-mps/mps-width");
  cfg_mps_approach_dist_ = config->get_float("/navgraph-generator-mps/approach-distance");

  navgen_if_ =
    blackboard->open_for_reading<NavGraphGeneratorInterface>("/navgraph-generator");

  navgen_mps_if_ =
    blackboard->open_for_writing<NavGraphWithMPSGeneratorInterface>("/navgraph-generator-mps");

  /* For testing
  tf::Quaternion q = tf::create_quaternion_from_yaw(deg2rad(45));
  double tag_pos[3] = {1.0,0.,0.};
  double tag_ori[4] = {q.x(), q.y(), q.z(), q.w()};
  update_station("Test", false, "/base_laser", tag_pos, tag_ori);
  generate_navgraph();
  */

  msg_waker_ = new BlackBoardOnMessageWaker(blackboard, navgen_mps_if_, this);
}

void
NavGraphGeneratorMPSThread::finalize()
{
  delete msg_waker_;

  blackboard->close(navgen_mps_if_);
  blackboard->close(navgen_if_);
}

void
NavGraphGeneratorMPSThread::loop()
{
  while ( ! navgen_mps_if_->msgq_empty() ) {
    if ( navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::ClearMessage>() ) {
      stations_.clear();

    } else if ( navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage>() ) {
      NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage *m =
	navgen_mps_if_->msgq_first<NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage>();
      logger->log_warn(name(), "Updating station %s from tag", m->id());

      update_station(m->id(), (m->side() == NavGraphWithMPSGeneratorInterface::INPUT),
		     m->frame(), m->tag_translation(), m->tag_rotation());

    } else if ( navgen_mps_if_->msgq_first_is<NavGraphWithMPSGeneratorInterface::ComputeMessage>() ) {
      generate_navgraph();
    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    navgen_mps_if_->msgq_pop();
  }
}


void
NavGraphGeneratorMPSThread::update_station(std::string id, bool input, std::string frame,
					   double tag_pos[3], double tag_ori[4])
 {
  // **** 1. convert to map frame
  tf::Stamped<tf::Pose> pose;

  try{
    // Note that we add a correction offset to the centroid X value.
    // This offset is determined empirically and just turns out to be helpful
    // in certain situations.

    tf::Stamped<tf::Pose>
      spose(tf::Pose(tf::Quaternion(tag_ori[0], tag_ori[1], tag_ori[2], tag_ori[3]),
                     tf::Vector3(tag_pos[0], tag_pos[1], tag_pos[2])),
            fawkes::Time(0,0), frame);
    tf_listener->transform_pose(cfg_global_frame_, spose, pose);
  } catch (tf::TransformException &e) {
    logger->log_warn(name(), "Transform exception:");
    logger->log_warn(name(), e);
    logger->log_error(name(), "Failed to add station %s: transform failed", id.c_str());
    return;
  }
  tf::Vector3     tf_pose_pos(pose.getOrigin());
  tf::Quaternion  tf_pose_ori(pose.getRotation());
  Eigen::Vector3f tag_pose_pos(tf_pose_pos.x(), tf_pose_pos.y(), tf_pose_pos.z());

  // **** 2. project to ground plane
  Eigen::Vector3f proj_pos;

  {
    Eigen::Vector3f plane_normal = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f plane_origin(0,0,0);
    Eigen::Vector3f po = tag_pose_pos - plane_origin;
    float lambda = plane_normal.dot(po);
    proj_pos = tag_pose_pos - (lambda * plane_normal);
  }

  Eigen::Quaternionf proj_ori(Eigen::AngleAxisf(tf::get_yaw(tf_pose_ori), Eigen::Vector3f::UnitZ()));

  // **** 3. calculate approach points
  Eigen::Vector3f dir_vec = proj_ori * Eigen::Vector3f::UnitX();
  Eigen::ParametrizedLine<float,3> mline(proj_pos, dir_vec);

  Eigen::Vector3f     input_pos, output_pos, pose_pos;
  Eigen::Quaternionf  input_ori, output_ori, pose_ori;
  if (input) {
    input_pos  = mline.pointAt(cfg_mps_approach_dist_);
    input_ori  = proj_ori * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
    output_pos = mline.pointAt(-(cfg_mps_approach_dist_ + cfg_mps_width_));
    output_ori = proj_ori;
    pose_pos   = mline.pointAt(-(.5 * cfg_mps_width_));
    pose_ori   = output_ori;
  } else {
    output_pos = mline.pointAt(cfg_mps_approach_dist_);
    output_ori = proj_ori * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
    input_pos  = mline.pointAt(-(cfg_mps_approach_dist_ + cfg_mps_width_));
    input_ori  = proj_ori;
    pose_pos   = mline.pointAt(-(.5 * cfg_mps_width_));
    pose_ori   = input_ori;
  }
  
  // Problem: range of first value is only [0..pi], which might result
  // in "funny" and unexpected angles. If we want only yaw, we can work
  // around this using rpy order... Better use utils.
  //Eigen::Vector3f input_euler  = input_ori.toRotationMatrix().eulerAngles(0, 1, 2);

  /*
  logger->log_info(name(), "P(%f,%f,%f) Q(%f,%f,%f,%f)  D(%f,%f,%f)",
		   proj_pos[0], proj_pos[1], proj_pos[2],
		   proj_ori.x(), proj_ori.y(), proj_ori.z(), proj_ori.w(),
		   dir_vec[0], dir_vec[1], dir_vec[2]);
  logger->log_info(name(), "I(%f,%f,%f)  O(%f,%f,%f)  DI %f  DO %f",
		   input_pos[0], input_pos[1], input_pos[2],
		   output_pos[0], output_pos[1], output_pos[2],
		   (input_pos - proj_pos).norm(), (output_pos - proj_pos).norm());
  */

  // **** 4. add to stations
  MPSStation s;
  s.tag_frame    = frame;
  s.tag_pose_pos = pose_pos;
  s.tag_pose_ori = proj_ori;
  s.pose_pos     = pose_pos;
  s.pose_ori     = pose_ori;
  s.input_pos    = input_pos;
  s.input_ori    = input_ori;
  s.input_yaw    = quat_yaw(input_ori);
  s.output_pos   = output_pos;
  s.output_ori   = output_ori;
  s.output_yaw   = quat_yaw(output_ori);
  stations_[id]  = s;
}


void
NavGraphGeneratorMPSThread::generate_navgraph()
{
  navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::ClearMessage());
  navgen_if_->msgq_enqueue(
    new NavGraphGeneratorInterface::SetBoundingBoxMessage(-7, -1, 7, 7));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetFilterMessage
     (NavGraphGeneratorInterface::FILTER_EDGES_BY_MAP, true));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetFilterMessage
     (NavGraphGeneratorInterface::FILTER_ORPHAN_NODES, true));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetFilterMessage
     (NavGraphGeneratorInterface::FILTER_MULTI_GRAPH, true));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetFilterParamFloatMessage
     (NavGraphGeneratorInterface::FILTER_EDGES_BY_MAP, "distance", 0.4));

  navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::AddMapObstaclesMessage(0.5));

  navgen_if_->msgq_enqueue
    (new NavGraphGeneratorInterface::SetCopyGraphDefaultPropertiesMessage(true));

  for (const auto &s : stations_) {
    navgen_if_->msgq_enqueue
      (new NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage
       ((s.first + "-I").c_str(),
	s.second.input_pos[0], s.second.input_pos[1], s.second.input_yaw,
	NavGraphGeneratorInterface::CLOSEST_EDGE));

    navgen_if_->msgq_enqueue
      (new NavGraphGeneratorInterface::AddPointOfInterestWithOriMessage
       ((s.first + "-O").c_str(),
	s.second.output_pos[0], s.second.output_pos[1], s.second.output_yaw,
	NavGraphGeneratorInterface::CLOSEST_EDGE));

    navgen_if_->msgq_enqueue
      (new NavGraphGeneratorInterface::AddObstacleMessage
       ((s.first + "-C").c_str(), s.second.pose_pos[0], s.second.pose_pos[1]));
  }

  navgen_if_->msgq_enqueue(new NavGraphGeneratorInterface::ComputeMessage());
}
