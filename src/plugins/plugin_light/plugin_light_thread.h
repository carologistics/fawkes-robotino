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
#include <aspect/tf.h>

#include <interfaces/PolarPosition2DInterface.h>

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

namespace fawkes {
	class PolarPosition2DInterface;
	namespace tf {
		class TransformListener;
	}
}

class PluginLightThread
:	public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::VisionAspect,
	public fawkes::BlackBoardAspect,
	public fawkes::TransformAspect
{

private:
	std::string cfg_prefix;

	std::string cfg_camera;
	float cfg_cameraAngleHorizontal;
	float cfg_cameraAngleVertical;
	unsigned int img_width;
	unsigned int img_height;

	std::string cfg_frame;

	unsigned int cfg_threasholdBrightness;
	float cfg_lightSizeWidth;
	float cfg_lightSizeHeight;

	bool cfg_debugMessages;

	firevision::Camera *camera;
	firevision::ScanlineModel *scanline;
	firevision::ColorModelBrightness *colorModel;
	firevision::SimpleColorClassifier *classifierLight;
	firevision::SharedMemoryImageBuffer *shmBufferYCbCr;
	unsigned char *bufferYCbCr;													//reference to the buffer of shm_buffer_YCbCr (to use in code)

	firevision::colorspace_t cspaceFrom;
	firevision::colorspace_t cspaceTo;

	fawkes::PolarPosition2DInterface *lightPositionLasterIF;

	unsigned char* calculatePositionInCamBuffer();

	std::list<firevision::ROI>* getROIs(unsigned char *buffer, unsigned int imgWidth, unsigned int imgHeight_);
	void drawLightIntoBuffer(fawkes::polar_coord_2d_t positionOfLight);
	fawkes::polar_coord_2d_t transformPolarCoord2D(fawkes::polar_coord_2d_t polFrom, std::string from, std::string to);

	void polToCart(fawkes::polar_coord_2d_t pol, float &x, float &y);
	void cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y);

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
