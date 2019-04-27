/***************************************************************************
 *  pipeline_thread.h - Robotino AmpelVar Pipeline Thread
 *
 *  Created: Thu May 24 17:17:46 2012
 *  Copyright  2005-2012  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
 *             2005       Martin Heracles
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

#include "machinedetection_thread.h"

#include <fvcams/camera.h>
#include <fvmodels/scanlines/grid.h>
#include <fvutils/color/conversions.h>
#include <fvutils/draw/drawer.h>
#include <fvutils/ipc/shm_image.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>
#include <utils/math/angle.h> //diff angle
#include <utils/math/coord.h> //cart2polar2d
#include <utils/system/hostinfo.h>

#include <cmath>
#include <cstdio>
#include <stdlib.h>
#include <string>

using namespace std;
using namespace fawkes;
using namespace firevision;

/** @class RobotinoMachineDetectionThread "machinedetection_thread.h"
 * Detect the ampel in the image by brightness
 *
 * @author Daniel Ewert
 */

/** Constructor. */
RobotinoMachineDetectionThread::RobotinoMachineDetectionThread()
: Thread("RobotinoMachineDetectionThread", Thread::OPMODE_WAITFORWAKEUP),
  VisionAspect(VisionAspect::CYCLIC)
{
}

/** Destructor. */
RobotinoMachineDetectionThread::~RobotinoMachineDetectionThread()
{
}

/** Initialize the pipeline thread.
 * Camera is requested, config parameters are set.
 */
void
RobotinoMachineDetectionThread::init()
{
	// receive a camera
	logger->log_debug(name(), "starting up");
	cam_ = vision_master->register_for_camera(
	  "v4l2:front:device=/dev/"
	  "video0:format=YUYV:size=160x120:white_balance_temperature_auto=0:"
	  "exposure_auto=1:exposure_absolute=305",
	  this);
	pic_width_  = cam_->pixel_width();
	pic_height_ = cam_->pixel_height();
	// pic_colorspace_ = YUV422_PLANAR;
	pic_colorspace_ = cam_->colorspace();

	full_image_roi_ = ROI::full_image(pic_width_, pic_height_);

	shm_buffer_ =
	  new SharedMemoryImageBuffer("machinedetection", pic_colorspace_, pic_width_, pic_height_);
	buffer_ = shm_buffer_->buffer();

	scanlinegrid_ = new ScanlineGrid(pic_width_, pic_height_, 5, 5, full_image_roi_);

	blobFarm = new BlobFarm(buffer_, pic_width_, pic_height_, 3, 20, 2, 200);
	logger->log_debug(name(), "init complete");
}

/** Thread finalization. */
void
RobotinoMachineDetectionThread::finalize()
{
}

/** Process image to detect objects.
 * Retrieves a new image from the camera and uses the classifier
 * determine objects according to the colormap. Publish the
 * position of the closest such object as puck position.
 */
void
RobotinoMachineDetectionThread::loop()
{
	cam_->capture();
	buffer_ = cam_->buffer();

	// Filtered
	logger->log_debug(name(), "Filtering done");
	scanlinegrid_->reset();
	while (!scanlinegrid_->finished()) {
		unsigned char pix = buffer_[(*scanlinegrid_)->y * pic_width_ + (*scanlinegrid_)->x];
		if (pix > 200) {
			logger->log_debug(name(), "pix(%d,%d):%d", (*scanlinegrid_)->x, (*scanlinegrid_)->y, pix);
		}
		if (pix > 200)
			blobFarm->add_Blob((*scanlinegrid_)->x, (*scanlinegrid_)->y);
		++(*scanlinegrid_);
	}
	logger->log_debug(name(), "Scanning done");
	std::list<Blob *> blobs = blobFarm->get_valid_blobs();
	for (std::list<Blob *>::iterator it = blobs.begin(); it != blobs.end(); ++it) {
		logger->log_debug(name(),
		                  "Blob:(%d|%d)/(%d|%d)",
		                  (*it)->getLeftX(),
		                  (*it)->getUpperY(),
		                  (*it)->getRightX(),
		                  (*it)->getLowerY());
	}

	ImageDisplay *dis = new ImageDisplay(pic_width_, pic_height_, "buffer_filtered");
	dis->show(pic_colorspace_, buffer_);
	dis->loop_until_quit();
}
