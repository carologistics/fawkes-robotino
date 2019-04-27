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

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/vision.h>
#include <core/threading/thread.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/RobotinoLightInterface.h>
#include <interfaces/SwitchInterface.h>

//#include <fvcams/camera.h>
#include <fvcams/fileloader.h>
#include <fvclassifiers/simple.h>
#include <fvfilters/roidraw.h>
#include <fvmodels/color/thresholds_black.h>
#include <fvmodels/color/thresholds_luminance.h>
#include <fvmodels/scanlines/grid.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <list>
#include <string>

namespace fawkes {
class Position3DInterface;
class RobotinoLightInterface;
namespace tf {
class TransformListener;
}
} // namespace fawkes

class LightFrontThread : public fawkes::Thread,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::VisionAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::TransformAspect
{
private:
	struct lightROIs
	{
		firevision::ROI light;
		firevision::ROI red;
		firevision::ROI yellow;
		firevision::ROI green;
	};

	struct lightSignal
	{
		fawkes::RobotinoLightInterface::LightState red;
		fawkes::RobotinoLightInterface::LightState yellow;
		fawkes::RobotinoLightInterface::LightState green;
		fawkes::polar_coord_2d_t                   nearestMaschine_pos;
		int                                        nearestMaschine_history;
	};

	//	struct light{
	//
	//		fawkes::RobotinoLightInterface interface_lightState;
	//		boost::circular_buffer<lightSignal> history_buffer;
	//	};

	std::string cfg_prefix;

	std::string  cfg_camera;
	float        cfg_cameraFactorHorizontal;
	float        cfg_cameraFactorVertical;
	float        cfg_cameraAngleVerticalRad;
	float        cfg_cameraAngleHorizontalRad;
	unsigned int img_width;
	unsigned int img_height;

	int cfg_cameraOffsetVertical;
	int cfg_cameraOffsetHorizontal;

	int cfg_lightNumberOfWrongDetections;

	float cfg_lightMoveUnderRfidThrashold;
	int   cfg_lightOutOfRangeThrashold;
	int   cfg_laserVisibilityThreshold;
	int   cfg_center_x;
	int   cfg_center_y;
	int   cfg_center_height;
	int   cfg_center_width;

	int   cfg_laserVisibilityThreashold;
	float cfg_lightDistanceAllowedBetweenFrames;

	int cfg_detectionCycleTime;
	int detectionCycleTimeFrames;

	int   lightOutOfRangeCounter;
	int   cfg_lightOutOfRangeThreshold;
	float cfg_lightToCloseThrashold;

	std::string cfg_frame;

	unsigned int cfg_brightnessThreshold;
	unsigned int cfg_darknessThreshold;
	float        cfg_lightSizeWidth;
	float        cfg_lightSizeHeight;

	float cfg_desiredLoopTime;

	bool  cfg_lightPositionCorrection;
	bool  cfg_debugMessagesActivated;
	bool  cfg_paintROIsActivated;
	bool  cfg_simulateLaserData;
	bool  cfg_useMinDistanceThreashold;
	bool  cfg_useMulticluster;
	float cfg_simulate_laser_x;
	float cfg_simulate_laser_y;
	int   cfg_simulate_laser_history;

	boost::circular_buffer<lightSignal> *historyBufferCluster1;
	boost::circular_buffer<lightSignal> *historyBufferCluster2;
	boost::circular_buffer<lightSignal> *historyBufferCluster3;
	// std::deque<lightSignal> *historyBufferCluster1;

	firevision::Camera *                 camera;
	firevision::ScanlineModel *          scanline;
	firevision::ColorModelLuminance *    colorModel;
	firevision::ColorModelBlack *        colorModelBlack;
	firevision::SimpleColorClassifier *  classifierWhite;
	firevision::SimpleColorClassifier *  classifierBlack;
	firevision::SharedMemoryImageBuffer *shmBufferYCbCr;
	unsigned char *bufferYCbCr; // reference to the buffer of shm_buffer_YCbCr (to
	                            // use in code)

	firevision::colorspace_t cspaceFrom;
	firevision::colorspace_t cspaceTo;

	fawkes::Position3DInterface *nearestMaschineIF1;
	fawkes::Position3DInterface *nearestMaschineIF2;
	fawkes::Position3DInterface *nearestMaschineIF3;

	fawkes::RobotinoLightInterface *lightStateIF1;
	fawkes::RobotinoLightInterface *lightStateIF2;
	fawkes::RobotinoLightInterface *lightStateIF3;

	fawkes::SwitchInterface *switchInterface;

	firevision::FilterROIDraw *drawer;

	LightFrontThread::lightSignal detectLightInCurrentPicture(LightFrontThread::lightROIs lightROIs);
	LightFrontThread::lightROIs correctLightRoisWithBlack(LightFrontThread::lightROIs expectedLight);
	fawkes::RobotinoLightInterface::LightState signalLightCurrentPicture(firevision::ROI signal);
	fawkes::RobotinoLightInterface::LightState signalLightWithHistory(int lightHistory,
	                                                                  int buffersize);
	std::list<firevision::ROI> *               classifyInRoi(firevision::ROI         searchArea,
	                                                         firevision::Classifier *classifier);
	bool lightFromHistoryBuffer(boost::circular_buffer<LightFrontThread::lightSignal> *buffer,
	                            LightFrontThread::lightSignal *                        lighSignal);

	unsigned char *calculatePositionInCamBuffer();

	int                         angleCorrectionY(float distance);
	int                         angleCorrectionX(float distance);
	LightFrontThread::lightROIs calculateLightPos(fawkes::polar_coord_2d_t lightPos);
	LightFrontThread::lightROIs createLightROIs(firevision::ROI light);

	bool isValidSuccessor(lightSignal previous, lightSignal current);

	//	void updateLocalHistory(LightFrontThread::lightSignal
	// lightSignalCurrent);

	void drawROIIntoBuffer(
	  firevision::ROI                           roi,
	  firevision::FilterROIDraw::border_style_t borderStyle = firevision::FilterROIDraw::DASHED_HINT);
	fawkes::polar_coord_2d_t
	transformCoordinateSystem(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to);

	void cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y);
	void polToCart(float &x, float &y, fawkes::polar_coord_2d_t pol);

	void resetLightInterface(fawkes::RobotinoLightInterface *interface, std::string message = "");
	void resetLocalHistory();
	void writeLightInterface(fawkes::RobotinoLightInterface *interface,
	                         LightFrontThread::lightSignal   lightSignal,
	                         bool                            ready);
	fawkes::cart_coord_3d_t getNearestMaschineFromInterface(fawkes::Position3DInterface *interface);

	void createUnknownLightSignal();

	bool isLightInViewarea(fawkes::polar_coord_2d_t light);
	void takePicture(LightFrontThread::lightROIs lightROIs);

	void            processHistoryBuffer(fawkes::RobotinoLightInterface *                       interface,
	                                     boost::circular_buffer<LightFrontThread::lightSignal> *buffer);
	firevision::ROI getBiggestRoi(std::list<firevision::ROI> *roiList);
	void            checkIfROIIsInBuffer(const firevision::ROI &light);
	void processSignal(fawkes::cart_coord_3d_t lightPosition, bool contiueToPictureProcess);
	void process_light(fawkes::Position3DInterface *                          position_interface,
	                   fawkes::RobotinoLightInterface *                       interface,
	                   boost::circular_buffer<LightFrontThread::lightSignal> *buffer);

protected:
	virtual void
	run()
	{
		Thread::run();
	}

public:
	LightFrontThread();
	virtual ~LightFrontThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();
};

#endif /* LIGHT_FRONT_THREAD_H_ */
