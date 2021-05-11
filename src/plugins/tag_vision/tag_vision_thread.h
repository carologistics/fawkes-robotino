/***************************************************************************
 *  tag_vision_thread.h - Thread to print the robot's position to the log
 *
 *  Created: Thu Sep 27 14:27:09 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_TAG_VISION_TAG_VISION_THREAD_H_
#define __PLUGINS_TAG_VISION_TAG_VISION_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/vision.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>

#include <vector>

// config handling
#include <config/change_handler.h>

// cv is needed for image conversion to alvar
#include <opencv2/opencv.hpp>
// alvar marker detection to get poses
#ifdef HAVE_AR_TRACK_ALVAR
#	include <ar_track_alvar/MarkerDetector.h>
#else
#	include <alvar/MarkerDetector.h>
#endif

// firevision camera
#include <fvcams/camera.h>
#include <fvclassifiers/simple.h>
#include <fvutils/adapters/cvmatadapter.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>

// interface
#include "tag_position_list.h"

#include <interfaces/LaserLineInterface.h>
#include <interfaces/TagVisionInterface.h>

#define MAX_MARKERS 16

namespace fawkes {
class Position3DInterface;
}

namespace firevision {
class Camera;
class SharedMemoryImageBuffer;
} // namespace firevision

class TagVisionThread : public fawkes::Thread,
                        public fawkes::LoggingAspect,
                        public fawkes::ConfigurableAspect,
                        public fawkes::BlackBoardAspect,
                        public fawkes::VisionAspect,
                        public fawkes::ConfigurationChangeHandler,
                        public fawkes::ClockAspect,
                        public fawkes::TransformAspect
{
public:
	TagVisionThread();
	// marker size accasors
	// thread functions
	virtual void init() override;
	virtual void loop() override;
	virtual void finalize() override;

	fawkes::tf::TransformPublisher *get_tf_publisher(size_t idx);

	/// Common prefix of all tag frame names
	static const std::string tag_frame_basename;

private:
	/// load config from file
	void loadConfig();
	/// the marker detector in alvar
	alvar::MarkerDetector<alvar::MarkerData> alvar_detector_;
	/// the camera the detector uses
	alvar::Camera alvar_cam_;
	/// the size of a marker in millimeter
	uint marker_size_;
	/// function to get the markers from an image
	void get_marker();
	/// store the alvar markers, containing the poses
	std::vector<alvar::MarkerData> *markers_;
	/// maximum markers to detect, size for the markers array
	size_t max_marker_;

	/// mutex for config access
	fawkes::Mutex cfg_mutex_;

	/// firevision camera
	firevision::Camera *fv_cam_;
	/// firevision image buffer
	firevision::SharedMemoryImageBuffer *shm_buffer_;
	unsigned char *                      image_buffer_;
	/// Image Buffer Id
	std::string shm_id_;

	// config handling
	virtual void config_value_erased(const char *path) override;
	virtual void config_tag_changed(const char *new_tag) override;
	virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v) override;
	virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v) override;

	/// cv image
	cv::Mat ipl_image_;

	/// blackboard communication
	TagPositionList *                          tag_interfaces_;
	std::vector<fawkes::LaserLineInterface *> *laser_line_ifs_;

	/// Width of the image
	unsigned int img_width_;
	/// Height of the image
	unsigned int img_height_;
};

#endif
