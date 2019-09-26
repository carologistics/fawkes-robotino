
/***************************************************************************
 *  plugin_light_thread.h - empty example
 *
 *  Created: Mi 23. Mai 17:44:14 CEST 2012
 *  Copyright  2012 Daniel Ewert
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

#ifndef __PLUGINS_LIGHT_COMPASS_THREAD_H_
#define __PLUGINS_LIGHT_COMPASS_THREAD_H_

#include "ColorModelRange.h"

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/vision.h>
#include <core/threading/thread.h>
#include <fvcams/camera.h>
#include <fvclassifiers/simple.h>
#include <fvfilters/roidraw.h>
#include <fvmodels/global_position/omni_global.h>
#include <fvmodels/mirror/bulb.h>
#include <fvmodels/mirror/mirrormodel.h>
#include <fvmodels/relative_position/omni_relative.h>
#include <fvmodels/scanlines/radial.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <fvutils/draw/drawer.h>
#include <fvutils/ipc/shm_image.h>
#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>
#include <interfaces/Position3DInterface.h>
#include <tf/transform_listener.h>
#include <tf/types.h>
#include <utils/math/coord.h>
#include <utils/system/hostinfo.h>

#include <string>

namespace fawkes {
class Position3DInterface;
}
namespace firevision {
class Camera;
class ScanlineModel;
class ColorModel;
class MirrorModel;
class RelativePositionModel;
class Bulb;
} // namespace firevision

class LightCompassThread : public fawkes::Thread,
                           public fawkes::LoggingAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::VisionAspect,
                           public fawkes::BlackBoardAspect,
                           public fawkes::TransformAspect
{
public:
	typedef fawkes::tf::Stamped<fawkes::tf::Point> Point3d;
	LightCompassThread();

	virtual void init();
	virtual void loop();
	virtual bool prepare_finalize_user();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	std::string cfg_prefix_;
	std::string cfg_camera_;
	std::string cfg_mirror_file_;
	std::string cfg_frame_;

	float cfg_mirror_height;
	float cfg_red_height_;
	float cfg_orange_height_;
	float cfg_green_height_;
	float cfg_robot_radius_;
	float cfg_lightDistanceAllowedBetweenFrames;

	bool cfg_useBulbFile_;
	bool cfg_debugOutput_;

	unsigned int cfg_threashold_roiMaxSize_;

	unsigned int cfg_color_y_min;
	unsigned int cfg_color_y_max;
	unsigned int cfg_color_u_min;
	unsigned int cfg_color_u_max;
	unsigned int cfg_color_v_min;
	unsigned int cfg_color_v_max;

	void                     drawROIsInBuffer(unsigned char *             buffer,
	                                          unsigned int                width,
	                                          unsigned int                height,
	                                          std::list<firevision::ROI> *rois);
	void                     drawROIInBuffer(unsigned char *  buffer,
	                                         unsigned int     width,
	                                         unsigned int     height,
	                                         firevision::ROI *roi);
	fawkes::polar_coord_2d_t mirrorCorrectionOfROI(firevision::ROI *roi);

	firevision::ROI *getBestROI(unsigned char *bufferYCbCr);
	fawkes::polar_coord_2d_t
	     distanceCorrectionOfAmple(fawkes::polar_coord_2d_t polatCooredOfMessuredLight, float ampleHeight);
	void UpdateInterface(double lightPosX, double lightPosY, std::string frame_id);
	void ResetInterface();
	bool isValidSuccessor(float px, float py, float cx, float cy);

	Point3d apply_tf_to_global(Point3d src);

	unsigned int img_width_;
	unsigned int img_height_;

	unsigned char *buffer_filtered;

	firevision::Camera *       cam_;
	firevision::ScanlineModel *scanline_;
	// firevision::ColorModel *cm_;
	firevision::MirrorModel *            mirror_;
	firevision::RelativePositionModel *  rel_pos_;
	firevision::SimpleColorClassifier *  classifierExpected_;
	firevision::SharedMemoryImageBuffer *shm_buffer_filtered;
	firevision::colorspace_t             cspace_from_;
	firevision::colorspace_t             cspace_to_;
	firevision::ColorModelRange *        colorModel_;

	firevision::FilterROIDraw *filterROIDraw;

	fawkes::Position3DInterface *lightif;
};

#endif
