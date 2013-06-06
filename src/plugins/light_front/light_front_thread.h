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

#ifndef LIGHT_FRONT_THREAD_H_
#define LIGHT_FRONT_THREAD_H_

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
#include <fvmodels/color/thresholds_luminance.h>
#include <fvmodels/color/thresholds_black.h>
#include <fvmodels/scanlines/grid.h>

#include <fvclassifiers/simple.h>

#include <fvutils/base/roi.h>
#include <fvfilters/roidraw.h>

#include <string>
#include <list>
#include <cmath>

#include <boost/circular_buffer.hpp>

namespace fawkes {
	class Position3DInterface;
	class RobotinoLightInterface;
	namespace tf {
		class TransformListener;
	}
}

class LightFrontThread
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

	struct lightSignal{
		fawkes::RobotinoLightInterface::LightState red;
		fawkes::RobotinoLightInterface::LightState yellow;
		fawkes::RobotinoLightInterface::LightState green;
		fawkes::polar_coord_2d_t nearestMaschine_pos;
		int nearestMaschine_history;
	};

	std::string cfg_prefix;

	std::string cfg_camera;
	float cfg_cameraFactorHorizontal;
	float cfg_cameraFactorVertical;
	unsigned int img_width;
	unsigned int img_height;

	int cfg_cameraOffsetVertical;
	float cfg_cameraOffsetHorizontalRad;

	int cfg_lightNumberOfWrongDetections;

	float cfg_lightMoveUnderRfidThrashold;
	int cfg_laserVisibilityThreashold;
	float cfg_lightDistanceAllowedBetweenFrames;

	int cfg_detectionCycleTime;
	int detectionCycleTimeFrames;

	int lightOutOfRangeCounter;
	int cfg_lightOutOfRangeThrashold;
	float cfg_lightToCloseThrashold;

	std::string cfg_frame;

	unsigned int cfg_brightnessThreashold;
	unsigned int cfg_darknessThreashold;
	float cfg_lightSizeWidth;
	float cfg_lightSizeHeight;

	float cfg_desiredLoopTime;

	bool cfg_lightPositionCorrection;
	bool cfg_debugMessagesActivated;
	bool cfg_paintROIsActivated;
	bool cfg_simulateLaserData;
	float cfg_simulate_laser_x;
	float cfg_simulate_laser_y;
	int cfg_simulate_laser_history;

	boost::circular_buffer<lightSignal> *historyBuffer;
	//std::deque<lightSignal> *historyBuffer;

	firevision::Camera *camera;
	firevision::ScanlineModel *scanline;
	firevision::ColorModelLuminance *colorModel;
	firevision::ColorModelBlack *colorModelBlack;
	firevision::SimpleColorClassifier *classifierWhite;
	firevision::SimpleColorClassifier *classifierBlack;
	firevision::SharedMemoryImageBuffer *shmBufferYCbCr;
	unsigned char *bufferYCbCr;													//reference to the buffer of shm_buffer_YCbCr (to use in code)

	firevision::colorspace_t cspaceFrom;
	firevision::colorspace_t cspaceTo;

	fawkes::Position3DInterface *nearestMaschineIF;

	int laser_visibilityHistoryThrashold;

	fawkes::RobotinoLightInterface *lightStateIF;

	firevision::FilterROIDraw *drawer;

	LightFrontThread::lightSignal detectLightInCurrentPicture(LightFrontThread::lightROIs lightROIs);
	LightFrontThread::lightROIs correctLightRoisWithBlack(LightFrontThread::lightROIs expectedLight);
	fawkes::RobotinoLightInterface::LightState signalLightCurrentPicture(firevision::ROI signal);
	fawkes::RobotinoLightInterface::LightState signalLightWithHistory(int lightHistory);
	std::list<firevision::ROI>* classifyInRoi(firevision::ROI searchArea, firevision::Classifier *classifier);
	bool lightFromHistoryBuffer(LightFrontThread::lightSignal &lighSignal);
	unsigned char* calculatePositionInCamBuffer();

	LightFrontThread::lightROIs calculateLightPos(fawkes::polar_coord_2d_t lightPos);
	LightFrontThread::lightROIs createLightROIs(firevision::ROI light);

	bool isValidSuccessor(lightSignal previous,lightSignal current);

//	void updateLocalHistory(LightFrontThread::lightSignal lightSignalCurrent);
	void resetLightInterface(std::string message="");

	void drawROIIntoBuffer(firevision::ROI roi, firevision::FilterROIDraw::border_style_t borderStyle = firevision::FilterROIDraw::DASHED_HINT);
	fawkes::polar_coord_2d_t transformCoordinateSystem(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to);

	void cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y);
	void polToCart(float &x, float &y, fawkes::polar_coord_2d_t pol);
	void resetLocalHistory();
	void writeLightInterface(LightFrontThread::lightSignal lightSignal, bool ready);
	void createUnknownLightSignal();
	fawkes::cart_coord_3d_t getNearestMaschineFromInterface();

	bool isLightInViewarea(fawkes::polar_coord_2d_t light);
	void takePicture(LightFrontThread::lightROIs lightROIs);

	void processHistoryBuffer();
	firevision::ROI getBiggestRoi(std::list<firevision::ROI>* roiList);
	void checkIfROIIsInBuffer(const firevision::ROI& light);

protected:
	virtual void run() { Thread::run(); }

public:
	LightFrontThread();
	virtual ~LightFrontThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();
};

#endif /* LIGHT_FRONT_THREAD_H_ */
