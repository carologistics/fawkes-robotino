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

#include <interfaces/Position3DInterface.h>
#include <interfaces/RobotinoLightInterface.h>

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
	class Position3DInterface;
	class RobotinoLightInterface;
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

	struct lightROIs{
		firevision::ROI light;
		firevision::ROI red;
		firevision::ROI yellow;
		firevision::ROI green;
	};

	std::string cfg_prefix;

	std::string cfg_camera;
	float cfg_cameraFactorHorizontal;
	float cfg_cameraFactorVertical;
	unsigned int img_width;
	unsigned int img_height;

	int cfg_cameraOffsetVertical;
	float cfg_cameraOffsetHorizontalRad;

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

	fawkes::Position3DInterface *lightPositionLasterIF;
	fawkes::RobotinoLightInterface *lightStateIF;

	firevision::FilterROIDraw *drawer;

	unsigned char* calculatePositionInCamBuffer();

	std::list<firevision::ROI>* getROIs(unsigned char *buffer, unsigned int imgWidth, unsigned int imgHeight_);
	std::list<firevision::ROI>* removeUnimportantROIs(std::list<firevision::ROI>* ROIs, firevision::ROI light);
	PluginLightThread::lightROIs calculateLightPos(fawkes::polar_coord_2d_t lightPos);

	void writeLightInterface(fawkes::RobotinoLightInterface::LightState red,
							 fawkes::RobotinoLightInterface::LightState yellow,
							 fawkes::RobotinoLightInterface::LightState green,
							 bool ready,
							 bool resetVisibilityHistory = false
							 );

	void drawROIIntoBuffer(firevision::ROI roi, firevision::FilterROIDraw::border_style_t borderStyle = firevision::FilterROIDraw::DASHED_HINT);
	fawkes::polar_coord_2d_t transformPolarCoord2D(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to);

//	void polToCart(fawkes::polar_coord_2d_t pol, float &x, float &y);
	void cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y);

	bool isSignalOn(firevision::ROI signal);

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
