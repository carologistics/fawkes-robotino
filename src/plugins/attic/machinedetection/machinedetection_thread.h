/***************************************************************************
 *  pipeline_thread.h - Robotino AmpelVision Pipeline Thread
 *
 *  Created: Thu May 24 17:15:10 2012
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

#ifndef __PLUGINS_ROBOTINO_AMPELVAR_PIPELINE_THREAD_H_
#define __PLUGINS_ROBOTINO_AMPELVAR_PIPELINE_THREAD_H_

#include "BlobFarm.h"

#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <core/threading/thread.h>
#include <fvfilters/threshold.h>
#include <fvmodels/scanlines/grid.h>
#include <fvutils/base/roi.h>
#include <fvutils/base/types.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/readers/jpeg.h>
#include <fvwidgets/image_display.h>
#include <utils/math/types.h>

#include <string>

namespace firevision {
class Camera;
class SharedMemoryImageBuffer;
} // namespace firevision
namespace fawkes {
class Position3DInterface;
}

class RobotinoMachineDetectionThread : public fawkes::Thread,
                                       public fawkes::LoggingAspect,
                                       public fawkes::VisionAspect,
                                       public fawkes::ConfigurableAspect,
                                       public fawkes::BlackBoardAspect,
                                       public fawkes::ClockAspect
{
public:
	RobotinoMachineDetectionThread();
	virtual ~RobotinoMachineDetectionThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

private:
	void detect_white_blobs();

	BlobFarm *                           blobFarm;
	firevision::Camera *                 cam_;
	firevision::SharedMemoryImageBuffer *shm_buffer_;
	unsigned char *                      buffer_;
	int                                  pic_width_;
	int                                  pic_height_;

	firevision::ROI *full_image_roi_;

	firevision::colorspace_t pic_colorspace_;

	firevision::Filter *thresholdFilter_;

	firevision::ScanlineModel *scanlinegrid_;

	// for debugging
	firevision::Reader *reader_;
};

#endif
