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

#ifndef PLUGIN_LIGHT_THREAD_H_
#define PLUGIN_LIGHT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/vision.h>

//#include <fvcams/camera.h>
#include <fvcams/fileloader.h>

#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>

#include <fvmodels/scanlines/grid.h>

#include <fvclassifiers/simple.h>

#include <fvutils/base/roi.h>
#include <fvfilters/roidraw.h>

#include <string>
#include <list>
#include <cmath>

#include "brightness.h"

class PluginLightThread
:	public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::VisionAspect,
	public fawkes::BlackBoardAspect
{

private:
	std::string cfg_prefix_;

	std::string cfg_camera_;
	double cfg_cameraAngle_horizontal;
	double cfg_cameraAngle_vertical;
	unsigned int img_width_;
	unsigned int img_height_;

	unsigned int camOffsetTop;
	unsigned int camOffsetBottom;
	unsigned int img_heightMinusOffset;

	std::string cfg_frame_;

	unsigned int cfg_threashold_brightness_;
	double cfg_lightSize_width;
	double cfg_lightSize_height;

	firevision::Camera *cam_;
	firevision::ScanlineModel *scanline_;
	firevision::ColorModelBrightness *colorModel_;
	firevision::SimpleColorClassifier *classifier_light_;
	firevision::SharedMemoryImageBuffer *shm_buffer_YCbCr;
	unsigned char *buffer_YCbCr;												//reference to the buffer of shm_buffer_YCbCr (to use in code)

	firevision::colorspace_t cspace_from_;
	firevision::colorspace_t cspace_to_;

	unsigned char* calculatePositionInCamBuffer();

	std::list<firevision::ROI>* getROIs(unsigned char *buffer, unsigned int imgWidth, unsigned int imgHeight_);

protected:
	virtual void run() { Thread::run(); }

public:
	PluginLightThread();
	virtual ~PluginLightThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();
};

#endif /* PLUGIN_LIGHT_THREAD_H_ */
