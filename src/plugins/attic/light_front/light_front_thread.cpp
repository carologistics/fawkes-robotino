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

#include "light_front_thread.h"

LightFrontThread::LightFrontThread()
: Thread("LightFrontThread", Thread::OPMODE_WAITFORWAKEUP), VisionAspect(VisionAspect::CYCLIC)
{
	cfg_prefix = "";
	cfg_camera = "";
	cfg_frame  = "";

	cfg_cameraFactorHorizontal = 0;
	cfg_cameraFactorVertical   = 0;

	cfg_cameraOffsetHorizontal = 0;
	cfg_cameraOffsetVertical   = 0;

	cfg_lightSizeHeight = 0;
	cfg_lightSizeWidth  = 0;

	cfg_lightNumberOfWrongDetections = 0;
	cfg_detectionCycleTime           = 0;

	cfg_desiredLoopTime    = 0;
	cfg_detectionCycleTime = 0;

	cfg_brightnessThreshold      = 0;
	cfg_darknessThreshold        = 0;
	cfg_paintROIsActivated       = false;
	cfg_useMinDistanceThreashold = false;
	cfg_useMulticluster          = false;

	cfg_lightOutOfRangeThreshold    = 0;
	cfg_laserVisibilityThreshold    = 0;
	cfg_lightMoveUnderRfidThrashold = 0;

	img_width  = 0;
	img_height = 0;

	detectionCycleTimeFrames = 0;

	historyBufferCluster1 = NULL;
	historyBufferCluster2 = NULL;
	historyBufferCluster3 = NULL;

	bufferYCbCr = NULL;

	cspaceFrom = firevision::YUV422_PLANAR;
	cspaceTo   = firevision::YUV422_PLANAR;

	camera          = NULL;
	scanline        = NULL;
	colorModel      = NULL;
	classifierWhite = NULL;
	classifierBlack = NULL;
	shmBufferYCbCr  = NULL;

	nearestMaschineIF1 = NULL;
	nearestMaschineIF2 = NULL;
	nearestMaschineIF3 = NULL;
	lightStateIF1      = NULL;
	lightStateIF2      = NULL;
	lightStateIF3      = NULL;

	drawer = NULL;

	cfg_debugMessagesActivated = false;

	cfg_lightDistanceAllowedBetweenFrames = 0;
	lightOutOfRangeCounter                = 0;
}

void
LightFrontThread::init()
{
	logger->log_info(name(), "light_front: starts up");

	lightOutOfRangeCounter = 0;

	cfg_prefix = "/plugins/light_front/";

	cfg_frame = config->get_string((cfg_prefix + "frame").c_str());

	cfg_camera                 = config->get_string((cfg_prefix + "camera").c_str());
	cfg_cameraFactorHorizontal = config->get_float((cfg_prefix + "camera_factor_horizontal").c_str());
	cfg_cameraFactorVertical   = config->get_float((cfg_prefix + "camera_factor_vertical").c_str());

	cfg_cameraOffsetHorizontal = config->get_int((cfg_prefix + "camera_offset_horizontal").c_str());
	cfg_cameraOffsetVertical   = config->get_int((cfg_prefix + "camera_offset_vertical").c_str());
	cfg_cameraAngleVerticalRad =
	  config->get_float((cfg_prefix + "camera_angle_vertical_rad").c_str());
	cfg_cameraAngleHorizontalRad =
	  config->get_float((cfg_prefix + "camera_angle_horizontal_rad").c_str());
	cfg_lightNumberOfWrongDetections =
	  config->get_int((cfg_prefix + "light_number_of_wrong_detections").c_str());

	cfg_debugMessagesActivated = config->get_bool((cfg_prefix + "show_debug_messages").c_str());
	cfg_paintROIsActivated     = config->get_bool((cfg_prefix + "draw_rois").c_str());

	cfg_brightnessThreshold = config->get_uint((cfg_prefix + "threashold_brightness").c_str());
	cfg_darknessThreshold   = config->get_uint((cfg_prefix + "threashold_black").c_str());
	cfg_laserVisibilityThreshold =
	  config->get_int((cfg_prefix + "threashold_laser_visibility").c_str());
	cfg_lightDistanceAllowedBetweenFrames =
	  config->get_float((cfg_prefix + "light_distance_allowed_betwen_frames").c_str());
	cfg_lightOutOfRangeThrashold =
	  config->get_int((cfg_prefix + "light_out_of_range_thrashold").c_str());
	cfg_lightMoveUnderRfidThrashold =
	  config->get_float((cfg_prefix + "light_move_under_rfid_thrashold").c_str());
	cfg_center_x      = config->get_int((cfg_prefix + "center_x").c_str());
	cfg_center_y      = config->get_int((cfg_prefix + "center_y").c_str());
	cfg_center_height = config->get_int((cfg_prefix + "center_height").c_str());
	cfg_center_width  = config->get_int((cfg_prefix + "center_width").c_str());

	cfg_lightToCloseThrashold = config->get_float((cfg_prefix + "light_to_close_thrashold").c_str());

	cfg_lightSizeHeight = config->get_float((cfg_prefix + "light_size_height").c_str());
	cfg_lightSizeWidth  = config->get_float((cfg_prefix + "light_size_width").c_str());

	cfg_detectionCycleTime   = config->get_int((cfg_prefix + "detection_cycle_time").c_str());
	cfg_desiredLoopTime      = config->get_float("/fawkes/mainapp/desired_loop_time") / 1000000;
	detectionCycleTimeFrames = cfg_detectionCycleTime / cfg_desiredLoopTime;

	cfg_useMinDistanceThreashold =
	  config->get_bool((cfg_prefix + "use_min_distance_threashold").c_str());
	cfg_useMulticluster = config->get_bool((cfg_prefix + "use_multicluster").c_str());

	cfg_lightPositionCorrection =
	  config->get_bool((cfg_prefix + "light_position_correction").c_str());
	cfg_simulateLaserData = config->get_bool((cfg_prefix + "simulate_laser").c_str());
	cfg_simulate_laser_x  = config->get_float((cfg_prefix + "simulate_laser_data_x").c_str());
	cfg_simulate_laser_y  = config->get_float((cfg_prefix + "simulate_laser_data_y").c_str());
	cfg_simulate_laser_history =
	  config->get_int((cfg_prefix + "simulate_laser_data_visibility").c_str());

	logger->log_debug(name(), "Camera offset horizontal: %i", cfg_cameraOffsetHorizontal);
	logger->log_debug(name(), "Camera offset vertical: %i", cfg_cameraOffsetVertical);

	logger->log_debug(name(), "Camera offset angle horizontal: %f", cfg_cameraAngleHorizontalRad);
	logger->log_debug(name(), "Camera offset angle vertical: %f", cfg_cameraAngleVerticalRad);

	logger->log_debug(name(), "Simulate laser: %b", cfg_simulateLaserData);

	std::string shmID = config->get_string((cfg_prefix + "shm_image_id").c_str());

	camera = vision_master->register_for_camera(cfg_camera.c_str(), this);

	img_width  = camera->pixel_width();
	img_height = camera->pixel_height();

	cspaceFrom = camera->colorspace();

	scanline   = new firevision::ScanlineGrid(img_width, img_height, 1, 1);
	colorModel = new firevision::ColorModelLuminance(cfg_brightnessThreshold);

	classifierWhite = new firevision::SimpleColorClassifier(scanline,   // scanmodel
	                                                        colorModel, // colorModel
	                                                        30,         // num_min_points
	                                                        0,          // box_extend
	                                                        false,      // upward
	                                                        2,          // neighberhoud_min_match
	                                                        0,          // grow_by
	                                                        firevision::C_WHITE // color
	);

	colorModelBlack = new firevision::ColorModelBlack(cfg_darknessThreshold);
	classifierBlack = new firevision::SimpleColorClassifier(scanline,        // scanmodel
	                                                        colorModelBlack, // colorModel
	                                                        30,              // num_min_points
	                                                        0,               // box_extend
	                                                        false,           // upward
	                                                        2,               // neighberhoud_min_match
	                                                        0,               // grow_by
	                                                        firevision::C_BLACK // color
	);

	// SHM image buffer
	shmBufferYCbCr =
	  new firevision::SharedMemoryImageBuffer(shmID.c_str(), cspaceTo, img_width, img_height);
	if (!shmBufferYCbCr->is_valid()) {
		throw fawkes::Exception("Shared memory segment not valid");
	}
	shmBufferYCbCr->set_frame_id(cfg_frame.c_str());

	bufferYCbCr = shmBufferYCbCr->buffer();

	// Create a ringbuffer with the size of the configured frame count
	historyBufferCluster1 = new boost::circular_buffer<lightSignal>(detectionCycleTimeFrames);
	historyBufferCluster2 = new boost::circular_buffer<lightSignal>(detectionCycleTimeFrames);
	historyBufferCluster3 = new boost::circular_buffer<lightSignal>(detectionCycleTimeFrames);
	// historyBuffer = new std::deque<lightSignal>();

	// open interfaces
	std::string position_3d_interface_name =
	  config->get_string((cfg_prefix + "light_position_if").c_str());
	nearestMaschineIF1 = blackboard->open_for_reading<fawkes::Position3DInterface>(
	  (position_3d_interface_name + "1").c_str());
	nearestMaschineIF2 = blackboard->open_for_reading<fawkes::Position3DInterface>(
	  (position_3d_interface_name + "2").c_str());
	nearestMaschineIF3 = blackboard->open_for_reading<fawkes::Position3DInterface>(
	  (position_3d_interface_name + "3").c_str());

	std::string light_interface_name = config->get_string((cfg_prefix + "light_state_if").c_str());
	lightStateIF1                    = blackboard->open_for_writing<fawkes::RobotinoLightInterface>(
    (light_interface_name + "1").c_str());
	lightStateIF2 = blackboard->open_for_writing<fawkes::RobotinoLightInterface>(
	  (light_interface_name + "2").c_str());
	lightStateIF3 = blackboard->open_for_writing<fawkes::RobotinoLightInterface>(
	  (light_interface_name + "3").c_str());

	try {
		switchInterface = blackboard->open_for_writing<fawkes::SwitchInterface>("light_front_switch");
		switchInterface->set_enabled(true);
		switchInterface->write();
	} catch (fawkes::Exception &e) {
		e.append("Opening switch interface for writing failed");
		throw;
	}

	// ROIs
	drawer = new firevision::FilterROIDraw();

	resetLightInterface(lightStateIF1);
	resetLightInterface(lightStateIF2);
	resetLightInterface(lightStateIF3);

	resetLocalHistory();

	logger->log_debug(name(), "end of init()");
}

void LightFrontThread::finalize() // TODO check if everthing gets deleted
{
	logger->log_debug(name(), "start to free memory");

	vision_master->unregister_thread(this);

	delete camera;
	delete scanline;
	delete colorModel;
	delete classifierWhite;
	delete shmBufferYCbCr;
	delete historyBufferCluster1;
	delete historyBufferCluster2;
	delete historyBufferCluster3;

	blackboard->close(nearestMaschineIF1);
	blackboard->close(nearestMaschineIF2);
	blackboard->close(nearestMaschineIF3);
	blackboard->close(lightStateIF1);
	blackboard->close(lightStateIF2);
	blackboard->close(lightStateIF3);

	blackboard->close(switchInterface);

	logger->log_info(name(), "ends");
}

void
LightFrontThread::process_light(fawkes::Position3DInterface *   position_interface,
                                fawkes::RobotinoLightInterface *light_interface,
                                boost::circular_buffer<LightFrontThread::lightSignal> *buffer)
{
	position_interface->read();
	bool                    contiueToPictureProcess = false;
	fawkes::cart_coord_3d_t lightPosition = getNearestMaschineFromInterface(position_interface);
	int                     clusterVisibilityHistory = position_interface->visibility_history();
	if (cfg_simulateLaserData) {
		lightPosition.x          = cfg_simulate_laser_x;
		lightPosition.y          = cfg_simulate_laser_y;
		clusterVisibilityHistory = cfg_simulate_laser_history;
	}
	fawkes::polar_coord_2d_t lightPositionPolar;
	if (clusterVisibilityHistory > cfg_laserVisibilityThreshold) {
		if (clusterVisibilityHistory > 0) {
			lightPositionPolar =
			  transformCoordinateSystem(lightPosition, position_interface->frame(), cfg_frame);
			if (isLightInViewarea(lightPositionPolar)) {
				lightOutOfRangeCounter  = 0;
				contiueToPictureProcess = true;
			} else {
				if (lightOutOfRangeCounter < cfg_lightOutOfRangeThreshold) {
					if (!buffer->empty()) {
						lightPositionPolar      = buffer->front().nearestMaschine_pos;
						contiueToPictureProcess = true;
					} else {
						resetLightInterface(light_interface, "light is out of range and buffer is empty");
						buffer->clear();
					}
				} else {
					resetLightInterface(light_interface, "light is to often out of range for camera");
					buffer->clear();
				}
			}
		} else if (!buffer->empty()) {
			lightPositionPolar      = buffer->front().nearestMaschine_pos;
			contiueToPictureProcess = true;
		} else {
			resetLightInterface(light_interface, "laser visibility is < 0 and buffer is empty");
			buffer->clear();
		}
	} else {
		// if the laser doen't see anything, use it as the robot is directy in front
		// of the light
		lightPositionPolar.r    = cfg_lightToCloseThrashold;
		lightPositionPolar.phi  = 0.0;
		contiueToPictureProcess = true;
		//		resetLightInterface("laser visibility lower than threashold");
		//		resetLocalHistory();
	}
	if (contiueToPictureProcess) {
		try {
			LightFrontThread::lightROIs lightROIs = calculateLightPos(lightPositionPolar);
			takePicture(lightROIs);
			// correct light position
			if (cfg_lightPositionCorrection) {
				lightROIs = correctLightRoisWithBlack(lightROIs);
				if (cfg_paintROIsActivated) {
					drawROIIntoBuffer(lightROIs.light);
					drawROIIntoBuffer(lightROIs.red);
					drawROIIntoBuffer(lightROIs.yellow);
					drawROIIntoBuffer(lightROIs.green);
				}
			}
			LightFrontThread::lightSignal lightSignalCurrentPicture =
			  detectLightInCurrentPicture(lightROIs);
			lightSignalCurrentPicture.nearestMaschine_pos     = lightPositionPolar;
			lightSignalCurrentPicture.nearestMaschine_history = clusterVisibilityHistory;
			buffer->push_front(lightSignalCurrentPicture);
			processHistoryBuffer(light_interface, buffer);
		} catch (fawkes::Exception &e) {
			resetLightInterface(light_interface, "ROI is outside of the buffer");
			buffer->clear();
		}
	}
}

void
LightFrontThread::loop()
{
	while (!switchInterface->msgq_empty()) {
		if (fawkes::SwitchInterface::DisableSwitchMessage *msg =
		      switchInterface->msgq_first_safe(msg)) {
			switchInterface->set_enabled(false);
			resetLocalHistory();
			resetLightInterface(lightStateIF1, "Switch disable message received");
			resetLightInterface(lightStateIF2, "Switch disable message received");
			resetLightInterface(lightStateIF3, "Switch disable message received");
		} else if (fawkes::SwitchInterface::EnableSwitchMessage *msg =
		             switchInterface->msgq_first_safe(msg)) {
			switchInterface->set_enabled(true);
			resetLocalHistory();
			resetLightInterface(lightStateIF1, "Switch enable message received");
			resetLightInterface(lightStateIF2, "Switch enable message received");
			resetLightInterface(lightStateIF3, "Switch enable message received");
		}
		switchInterface->msgq_pop();
		switchInterface->write();
	}

	if (!switchInterface->is_enabled()) {
		return;
	}

	// read laser if
	nearestMaschineIF1->read();
	process_light(nearestMaschineIF1, lightStateIF1, historyBufferCluster1);

	if (cfg_useMulticluster) {
		nearestMaschineIF2->read();
		nearestMaschineIF3->read();
		process_light(nearestMaschineIF2, lightStateIF2, historyBufferCluster2);
		process_light(nearestMaschineIF3, lightStateIF3, historyBufferCluster3);
	}
}

bool
LightFrontThread::isLightInViewarea(fawkes::polar_coord_2d_t light)
{
	float cameraAngleDetectionArea = (cfg_cameraFactorHorizontal / 2) - 0.1;

	if (std::abs(light.phi) >= cameraAngleDetectionArea && light.r < cfg_lightToCloseThrashold) {
		return false;
	} else {
		return true;
	}
}

void
LightFrontThread::takePicture(LightFrontThread::lightROIs lightROIs)
{
	camera->capture();
	firevision::convert(cspaceFrom, cspaceTo, camera->buffer(), bufferYCbCr, img_width, img_height);
	camera->dispose_buffer();

	// draw expected light in buffer
	if (cfg_paintROIsActivated) {
		drawROIIntoBuffer(lightROIs.light);
		drawROIIntoBuffer(lightROIs.red);
		drawROIIntoBuffer(lightROIs.yellow);
		drawROIIntoBuffer(lightROIs.green);
	}
}

firevision::ROI
LightFrontThread::getBiggestRoi(std::list<firevision::ROI> *roiList)
{
	firevision::ROI *biggestRoi = new firevision::ROI();

	for (std::list<firevision::ROI>::iterator it = roiList->begin(); it != roiList->end(); ++it) {
		if (biggestRoi == NULL || biggestRoi->width * biggestRoi->height < (*it).width * (*it).height) {
			biggestRoi = &(*it);
		}
	}
	return biggestRoi;
}

LightFrontThread::lightROIs
LightFrontThread::correctLightRoisWithBlack(LightFrontThread::lightROIs expectedLight)
{
	// Look in area around the red light for the back top of the light

	firevision::ROI *searchAreaBottem = new firevision::ROI(expectedLight.light);
	searchAreaBottem->start.x         = expectedLight.light.start.x - 3 * expectedLight.light.width;
	searchAreaBottem->start.y =
	  expectedLight.light.start.y + expectedLight.light.height - expectedLight.light.width;
	searchAreaBottem->width  = expectedLight.light.width * 7;
	searchAreaBottem->height = 2 * expectedLight.light.width;

	if (cfg_paintROIsActivated) {
		drawROIIntoBuffer(searchAreaBottem);
	}

	std::list<firevision::ROI> *bottemBlackRoiList = classifyInRoi(searchAreaBottem, classifierBlack);

	if (!bottemBlackRoiList->empty()) {
		firevision::ROI bottemBlackRoi = getBiggestRoi(bottemBlackRoiList);

		drawROIIntoBuffer(bottemBlackRoi);

		if (cfg_debugMessagesActivated) {
			logger->log_debug(name(),
			                  "Bottem: X: %u Y: %u Height: %u Width: %u",
			                  bottemBlackRoi.start.x,
			                  bottemBlackRoi.start.y,
			                  bottemBlackRoi.height,
			                  bottemBlackRoi.width);
		}

		// Look in the area around the Red light for a black roi (top cap) for
		// validation
		firevision::ROI *searchAreaTop = new firevision::ROI(bottemBlackRoi);

		searchAreaTop->start.y = expectedLight.light.start.y - 2 * (expectedLight.light.height / 3);
		searchAreaTop->height  = expectedLight.green.height * 6;

		std::list<firevision::ROI> *topBlackRoiList = classifyInRoi(searchAreaTop, classifierBlack);
		if (!topBlackRoiList->empty()) {
			firevision::ROI topBiggestRoi = getBiggestRoi(topBlackRoiList);
			drawROIIntoBuffer(topBiggestRoi);

			if (cfg_debugMessagesActivated) {
				logger->log_debug(name(),
				                  "Top: X: %u Y: %u Height: %u Width: %u",
				                  topBiggestRoi.start.x,
				                  topBiggestRoi.start.y,
				                  topBiggestRoi.height,
				                  topBiggestRoi.width);
			}
			firevision::ROI light;
			light.start.x = topBiggestRoi.start.x;
			light.start.y = topBiggestRoi.start.y + topBiggestRoi.height;
			light.height  = bottemBlackRoi.start.y - (topBiggestRoi.start.y + topBiggestRoi.height);
			light.width   = std::min(bottemBlackRoi.width, topBiggestRoi.width);

			checkIfROIIsInBuffer(light);
			expectedLight = createLightROIs(light);
		} else {
			logger->log_debug(name(), "No black-bottom roi found");
		}
	} else {
		logger->log_debug(name(), "No black roi found");
	}

	return expectedLight;
}

LightFrontThread::lightSignal
LightFrontThread::detectLightInCurrentPicture(LightFrontThread::lightROIs lightROIs)
{
	LightFrontThread::lightSignal lightSignal;

	lightSignal.red    = signalLightCurrentPicture(lightROIs.red);
	lightSignal.yellow = signalLightCurrentPicture(lightROIs.yellow);
	lightSignal.green  = signalLightCurrentPicture(lightROIs.green);

	return lightSignal;
}

void LightFrontThread::processHistoryBuffer(
  fawkes::RobotinoLightInterface *interface,
  boost::circular_buffer<LightFrontThread::lightSignal>
    *buffer) // TODO change function name to say that the interface is writen
{
	if (historyBufferCluster1->full()) {
		LightFrontThread::lightSignal lighSignal;

		if (lightFromHistoryBuffer(buffer, &lighSignal)) {
			if (lighSignal.red != fawkes::RobotinoLightInterface::UNKNOWN
			    && lighSignal.yellow != fawkes::RobotinoLightInterface::UNKNOWN
			    && lighSignal.green != fawkes::RobotinoLightInterface::UNKNOWN
			    && !(lighSignal.red == fawkes::RobotinoLightInterface::OFF
			         && lighSignal.yellow == fawkes::RobotinoLightInterface::OFF
			         && lighSignal.green == fawkes::RobotinoLightInterface::OFF)) {
				writeLightInterface(interface, lighSignal, true);

			} else {
				resetLightInterface(interface, "light couldn't get detected");
			}
		} else {
			resetLightInterface(interface, "cluster jumped");
		}
	} else {
		writeLightInterface(interface, buffer->front(), false);
	}
}

LightFrontThread::~LightFrontThread()
{
	// TODO Auto-generated destructor stub
}

fawkes::polar_coord_2d_t
LightFrontThread::transformCoordinateSystem(fawkes::cart_coord_3d_t cartFrom,
                                            std::string             from,
                                            std::string             to)
{
	fawkes::polar_coord_2d_t polErrorReturnValue;
	cartToPol(polErrorReturnValue, cartFrom.x, cartFrom.y);

	bool world_frame_exists = tf_listener->frame_exists(from);
	bool robot_frame_exists = tf_listener->frame_exists(to);

	if (!world_frame_exists || !robot_frame_exists) {
		logger->log_warn(name(),
		                 "Frame missing: %s %s   %s %s",
		                 from.c_str(),
		                 world_frame_exists ? "exists" : "missing",
		                 to.c_str(),
		                 robot_frame_exists ? "exists" : "missing");
	} else {
		fawkes::tf::StampedTransform transform;
		try {
			tf_listener->lookup_transform(to, from, transform);
		} catch (fawkes::tf::ExtrapolationException &e) {
			logger->log_debug(name(), "Extrapolation error");
			return polErrorReturnValue;
		} catch (fawkes::tf::ConnectivityException &e) {
			logger->log_debug(name(), "Connectivity exception: %s", e.what());
			return polErrorReturnValue;
		}

		fawkes::tf::Vector3 v = transform.getOrigin();

		fawkes::polar_coord_2d_t polTo;
		float                    toX, toY;

		toX = cartFrom.x + v.getX();
		toY = cartFrom.y + v.getY();
		cartToPol(polTo, toX, toY);

		if (cfg_debugMessagesActivated) {
			logger->log_debug(name(), "From: %s X: %f Y: %f", from.c_str(), cartFrom.x, cartFrom.y);
			logger->log_debug(name(), "To  : %s X: %f Y: %f", to.c_str(), toX, toY);
		}
		return polTo;
	}

	return polErrorReturnValue;
}

void
LightFrontThread::cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y)
{
	pol.phi = atan2f(y, x);
	pol.r   = sqrtf(x * x + y * y);

	if (cfg_debugMessagesActivated) {
		logger->log_debug(name(), "x: %f; y: %f", x, y);
		logger->log_debug(name(), "Calculated r: %f; phi: %f", pol.r, pol.phi);
	}
}

void
LightFrontThread::polToCart(float &x, float &y, fawkes::polar_coord_2d_t pol)
{
	x = pol.r * std::cos(pol.phi);
	y = pol.r * std::sin(pol.phi);
}

void
LightFrontThread::drawROIIntoBuffer(firevision::ROI                           roi,
                                    firevision::FilterROIDraw::border_style_t borderStyle)
{
	drawer->set_src_buffer(bufferYCbCr, firevision::ROI::full_image(img_width, img_height), 0);
	drawer->set_dst_buffer(bufferYCbCr, &roi);
	drawer->set_style(borderStyle);
	drawer->apply();

	if (cfg_debugMessagesActivated) {
		logger->log_debug(name(), "drawed element in buffer");
	}
}

void
LightFrontThread::checkIfROIIsInBuffer(const firevision::ROI &light)
{
	if (light.start.x >= img_width || light.start.y >= img_height
	    || light.height + light.start.y >= img_height || light.width + light.start.x >= img_width) {
		throw fawkes::Exception("ROI is outsite of the buffer");
	} else {
		if (cfg_debugMessagesActivated) {
			logger->log_info(name(), "Size check ok");
		}
	}
}

int
LightFrontThread::angleCorrectionY(float distance)
{
	return img_height / (distance * cfg_cameraFactorVertical) * tan(cfg_cameraAngleVerticalRad);
}

int
LightFrontThread::angleCorrectionX(float distance)
{
	return img_width / (distance * cfg_cameraFactorHorizontal) * tan(cfg_cameraAngleHorizontalRad);
}

LightFrontThread::lightROIs
LightFrontThread::calculateLightPos(fawkes::polar_coord_2d_t lightPos)
{
	int expectedLightSizeWidth =
	  img_width / (lightPos.r * cfg_cameraFactorHorizontal) * cfg_lightSizeWidth;
	int expectedLightSizeHeigth =
	  img_height / (lightPos.r * cfg_cameraFactorVertical) * cfg_lightSizeHeight;
	float pixelPerRadHorizonal = img_width / cfg_cameraFactorHorizontal;

	int startX = img_width / 2                         // picture center
	             - lightPos.phi * pixelPerRadHorizonal // move to the light
	             - expectedLightSizeWidth / 2;

	int startY =
	  img_height / 2                 // picture center
	  - expectedLightSizeHeigth / 2; // light center to light cornor;  //light center to light cornor

	startY = startY + cfg_cameraOffsetVertical // error of picture position to light
	         + angleCorrectionY(lightPos.r);

	if (cfg_useMinDistanceThreashold && cfg_lightMoveUnderRfidThrashold >= lightPos.r) {
		startX                  = cfg_center_x;
		startY                  = cfg_center_y;
		expectedLightSizeHeigth = cfg_center_height;
		expectedLightSizeWidth  = cfg_center_width;
	}

	firevision::ROI light;
	light.start.x = startX;
	light.start.y = startY;
	light.height  = expectedLightSizeHeigth;
	light.width   = expectedLightSizeWidth;

	checkIfROIIsInBuffer(light);

	return createLightROIs(light);
}

LightFrontThread::lightROIs
LightFrontThread::createLightROIs(firevision::ROI light)
{
	LightFrontThread::lightROIs lightROIs;

	// light ROI size
	lightROIs.light              = light;
	lightROIs.light.image_height = img_height;
	lightROIs.light.image_width  = img_width;

	// Signale ROIs

	lightROIs.red    = lightROIs.light;
	lightROIs.yellow = lightROIs.light;
	lightROIs.green  = lightROIs.light;

	float roiHeight = ((float)lightROIs.light.height) / 9;

	lightROIs.red.height = roiHeight;
	lightROIs.red.start.y += roiHeight; // Middle of the top thirds

	lightROIs.yellow.height = roiHeight;
	lightROIs.yellow.start.y += roiHeight * 4; // Middle of the middle thirds

	lightROIs.green.height = roiHeight;
	lightROIs.green.start.y += roiHeight * 7; // Middle of the bottom thirds

	if (cfg_debugMessagesActivated) {
		logger->log_info(name(),
		                 "light.start.x %u, light.start.y %u, height: %u",
		                 lightROIs.light.start.x,
		                 lightROIs.light.start.y,
		                 lightROIs.light.height);
		logger->log_info(name(),
		                 "red.start.x %u, red.start.y %u, red: %u",
		                 lightROIs.red.start.x,
		                 lightROIs.red.start.y,
		                 lightROIs.red.height);
		logger->log_info(name(),
		                 "yellow.start.x %u, yellow.start.y %u, yellow: %u",
		                 lightROIs.yellow.start.x,
		                 lightROIs.yellow.start.y,
		                 lightROIs.yellow.height);
		logger->log_info(name(),
		                 "green.start.x %u, green.start.y %u, green: %u",
		                 lightROIs.green.start.x,
		                 lightROIs.green.start.y,
		                 lightROIs.green.height);
	}

	return lightROIs;
}

void
LightFrontThread::writeLightInterface(fawkes::RobotinoLightInterface *interface,
                                      LightFrontThread::lightSignal   lightSignal,
                                      bool                            ready)
{
	lightStateIF1->read();
	int vis = lightStateIF1->visibility_history();

	if (vis < 0 || lightStateIF1->red() != lightSignal.red
	    || lightStateIF1->yellow() != lightSignal.yellow
	    || lightStateIF1->green() != lightSignal.green) {
		vis   = 1;
		ready = false;
	} else {
		vis++;
	}

	lightStateIF1->set_visibility_history(vis);

	lightStateIF1->set_red(lightSignal.red);
	lightStateIF1->set_yellow(lightSignal.yellow);
	lightStateIF1->set_green(lightSignal.green);

	if (vis >= detectionCycleTimeFrames) {
		lightStateIF1->set_ready(ready);
	} else {
		lightStateIF1->set_ready(false);
	}

	lightStateIF1->write();
}

void
LightFrontThread::resetLocalHistory()
{
	historyBufferCluster1->clear();
	historyBufferCluster2->clear();
	historyBufferCluster3->clear();
}

void
LightFrontThread::resetLightInterface(fawkes::RobotinoLightInterface *interface,
                                      std::string                     message)
{
	interface->read();
	int vis = interface->visibility_history();

	if (vis < 0) {
		vis--;
	} else {
		vis = -1;
	}
	interface->set_visibility_history(vis);

	interface->set_ready(false);
	interface->write();

	if (cfg_debugMessagesActivated) {
		logger->log_info(name(), "Resetting interface, %s", message.c_str());
	}
}

std::list<firevision::ROI> *
LightFrontThread::classifyInRoi(firevision::ROI searchArea, firevision::Classifier *classifier)
{
	scanline->reset();
	scanline->set_roi(&searchArea);

	classifier->set_src_buffer(bufferYCbCr, img_width, img_height);

	std::list<firevision::ROI> *ROIs = classifier->classify();

	return ROIs;
}

fawkes::RobotinoLightInterface::LightState
LightFrontThread::signalLightCurrentPicture(firevision::ROI signal)
{
	scanline->reset();
	scanline->set_roi(&signal);

	classifierWhite->set_src_buffer(bufferYCbCr, img_width, img_height);

	std::list<firevision::ROI> *ROIs = classifierWhite->classify();

	bool isOn = !ROIs->empty();

	if (cfg_debugMessagesActivated) {
		int countedROIs = (int)ROIs->size();
		logger->log_debug(name(), "Detect: %i", countedROIs);
	}
	if (cfg_paintROIsActivated) {
		int countedROIs = (int)ROIs->size();
		for (int i = 0; i < countedROIs; ++i) {
			drawROIIntoBuffer(ROIs->front());
			ROIs->pop_front();
		}
	}

	delete ROIs;

	if (isOn) {
		return fawkes::RobotinoLightInterface::ON;
	} else {
		return fawkes::RobotinoLightInterface::OFF;
	}
}

bool
LightFrontThread::lightFromHistoryBuffer(
  boost::circular_buffer<LightFrontThread::lightSignal> *buffer,
  LightFrontThread::lightSignal *                        lighSignal)
{
	int red    = 0;
	int yellow = 0;
	int green  = 0;

	LightFrontThread::lightSignal previousLight;
	for (boost::circular_buffer<LightFrontThread::lightSignal>::iterator it = buffer->begin();
	     it != buffer->end();
	     ++it) {
		if (it != buffer->begin()) {
			if (!isValidSuccessor(*it, previousLight)) {
				return false;
			}
		}
		if (it->red == fawkes::RobotinoLightInterface::ON) {
			red++;
		}

		if (it->yellow == fawkes::RobotinoLightInterface::ON) {
			yellow++;
		}

		if (it->green == fawkes::RobotinoLightInterface::ON) {
			green++;
		}

		previousLight = *it;
	}

	lighSignal->red    = signalLightWithHistory(red, buffer->size());
	lighSignal->yellow = signalLightWithHistory(yellow, buffer->size());
	lighSignal->green  = signalLightWithHistory(green, buffer->size());

	return true;
}

bool
LightFrontThread::isValidSuccessor(lightSignal previous, lightSignal current)
{
	float px, py;
	float cx, cy;
	float dx, dy;

	polToCart(px, py, previous.nearestMaschine_pos);
	polToCart(cx, cy, current.nearestMaschine_pos);
	dx = cx - px;
	dy = cy - py;

	if (std::sqrt((dx * dx) + (dy * dy)) < cfg_lightDistanceAllowedBetweenFrames) {
		return true;
	}
	return false;
}

fawkes::RobotinoLightInterface::LightState
LightFrontThread::signalLightWithHistory(int lightHistory, int buffer_size)
{
	int visibilityHistory = buffer_size; // historyBufferCluster1->size();

	if (lightHistory >= (visibilityHistory - cfg_lightNumberOfWrongDetections)) {
		return fawkes::RobotinoLightInterface::ON;

	} else if (lightHistory >= visibilityHistory / 2 - cfg_lightNumberOfWrongDetections
	           && lightHistory <= visibilityHistory / 2 + cfg_lightNumberOfWrongDetections) {
		return fawkes::RobotinoLightInterface::BLINKING;

	} else if (lightHistory <= cfg_lightNumberOfWrongDetections) {
		return fawkes::RobotinoLightInterface::OFF;
	} else {
		if (cfg_debugMessagesActivated) {
			logger->log_info(name(),
			                 "Detected: %i ON out of %i Pictures (ON: %i OFF: %i BLINKING: %i)",
			                 lightHistory,
			                 visibilityHistory,
			                 visibilityHistory,
			                 0,
			                 visibilityHistory / 2);
		}
		return fawkes::RobotinoLightInterface::UNKNOWN;
	}
}

fawkes::cart_coord_3d_t
LightFrontThread::getNearestMaschineFromInterface(fawkes::Position3DInterface *interface)
{
	fawkes::cart_coord_3d_t lightPosition;

	interface->read();
	lightPosition.x = interface->translation(0);
	lightPosition.y = interface->translation(1);
	lightPosition.z = interface->translation(2);

	// if the light is directly in front (by x coordinate) set y to 0 because it
	// is likly to detecd wrong values
	if (lightPosition.x < cfg_lightMoveUnderRfidThrashold) {
		lightPosition.y = 0.0;
	} else {
		if (cfg_debugMessagesActivated) {
			logger->log_debug(name(), "Not a close mashine");
		}
	}

	return lightPosition;
}
