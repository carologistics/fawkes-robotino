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

#ifndef puck_vision_THREAD_H_
#define puck_vision_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/vision.h>
#include <aspect/tf.h>

#include <interfaces/Position3DInterface.h>

//#include <fvcams/camera.h>
#include <fvcams/fileloader.h>

#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include "ColorModelRange.h"
#include <fvmodels/scanlines/grid.h>

#include <fvclassifiers/simple.h>

#include <fvutils/base/roi.h>
#include <fvfilters/roidraw.h>

#include <string>
#include <list>
#include <cmath>

namespace fawkes {
	class Position3DInterface;
	namespace tf {
		class TransformListener;
	}
}

class PuckVisionThread
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
	float cfg_cameraFactorHorizontal;
	float cfg_cameraFactorVertical;
	unsigned int img_width;
	unsigned int img_height;

	int cfg_cameraOffsetVertical;
	float cfg_cameraOffsetHorizontalRad;

	float cfg_width_top_in_m;
	float cfg_width_bottem_in_m;

	float m_per_pixel_height;


	unsigned int cfg_color_y_min;
	unsigned int cfg_color_y_max;
	unsigned int cfg_color_u_min;
	unsigned int cfg_color_u_max;
	unsigned int cfg_color_v_min;
	unsigned int cfg_color_v_max;

	float cfg_puck_radius;
	float cfg_distance_function_a;
	float cfg_distance_function_b;

	bool cfg_debugMessagesActivated;
	bool cfg_paintROIsActivated;
	std::string cfg_frame;

	firevision::Camera *camera;
	firevision::ScanlineModel *scanline;
	firevision::ColorModelRange *colorModel;
	firevision::SimpleColorClassifier *classifierExpected;
	firevision::SharedMemoryImageBuffer *shmBufferYCbCr;
	firevision::ROI roi_center;
	unsigned char *bufferYCbCr;													//reference to the buffer of shm_buffer_YCbCr (to use in code)

	firevision::colorspace_t cspaceFrom;
	firevision::colorspace_t cspaceTo;

	fawkes::Position3DInterface *nearestPuck;
	firevision::FilterROIDraw *drawer;

	void drawROIIntoBuffer(firevision::ROI roi, firevision::FilterROIDraw::border_style_t borderStyle = firevision::FilterROIDraw::DASHED_HINT);
	fawkes::polar_coord_2d_t transformCoordinateSystem(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to);

	fawkes::polar_coord_2d_t positionFromRoi(firevision::ROI* roi);
	void cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y);
	void polToCart(float &x, float &y, fawkes::polar_coord_2d_t pol);

	double distanceCorrection(unsigned int x_in);
	double positionCorrectionY(firevision::ROI* roi);

	firevision::ROI* getBiggestRoi(std::list<firevision::ROI>* roiList);
	std::list<firevision::ROI>*	classifyInRoi(firevision::ROI searchArea, firevision::Classifier *classifier);
	void updateInterface(firevision::ROI* puck);


protected:
	virtual void run() { Thread::run(); }

public:
	PuckVisionThread();
	virtual ~PuckVisionThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();
};

#endif /* puck_vision_THREAD_H_ */
