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

#include "plugin_light_thread.h"

PluginLightThread::PluginLightThread()
:	Thread("PluginLightThread", Thread::OPMODE_WAITFORWAKEUP),
 	VisionAspect(VisionAspect::CYCLIC)
{
	this->cfg_prefix = "";
	this->cfg_camera = "";
	this->cfg_frame = "";

	this->cfg_cameraFactorHorizontal = 0;
	this->cfg_cameraFactorVertical = 0;

	this->cfg_lightSizeHeight = 0;
	this->cfg_lightSizeWidth = 0;

	this->cfg_lightNumberOfWrongDetections = 0;
	this->cfg_detectionCycleTime = 0;

	this->cfg_brightnessThreashold = 0;
	this->cfg_darknessThreashold = 0;
	this->cfg_paintROIsActivated = false;

	this->cfg_lightOutOfRangeThrashold = 0;

	this->img_width = 0;
	this->img_height = 0;

	this->bufferYCbCr = NULL;

	this->cspaceFrom = firevision::YUV422_PLANAR;
	this->cspaceTo = firevision::YUV422_PLANAR;

	this->camera = NULL;
	this->scanline = NULL;
	this->colorModel = NULL;
	this->classifierWhite = NULL;
	this->classifierBlack = NULL;
	this->shmBufferYCbCr = NULL;

	this->nearestMaschineIF = NULL;
	this->lightStateIF = NULL;

	this->drawer = NULL;

	this->cfg_debugMessagesActivated = false;

	this->laser_visibilityHistoryThrashold = 10;
	this->cfg_lightDistanceAllowedBetweenFrames = 0;
	this->lightOutOfRangeCounter = 0;
}

void
PluginLightThread::init()
{
	logger->log_info(name(), "Plugin-light: starts up");

	this->lightOutOfRangeCounter = 0;

	this->cfg_prefix = "/plugins/plugin_light/";

	this->cfg_frame  = this->config->get_string((this->cfg_prefix + "frame").c_str());

	this->cfg_camera = this->config->get_string((this->cfg_prefix + "camera").c_str());
	this->cfg_cameraFactorHorizontal = this->config->get_float((this->cfg_prefix + "camera_factor_horizontal").c_str());
	this->cfg_cameraFactorVertical = this->config->get_float((this->cfg_prefix + "camera_factor_vertical").c_str());

	this->cfg_cameraOffsetHorizontalRad = this->config->get_float((this->cfg_prefix + "camera_offset_horizontal_rad").c_str());
	this->cfg_cameraOffsetVertical = this->config->get_int((this->cfg_prefix + "camera_offset_vertical").c_str());

	this->cfg_lightNumberOfWrongDetections = this->config->get_int((this->cfg_prefix + "light_number_of_wrong_detections").c_str());

	this->cfg_debugMessagesActivated = this->config->get_bool((this->cfg_prefix + "show_debug_messages").c_str());
	this->cfg_paintROIsActivated = this->config->get_bool((this->cfg_prefix + "draw_rois").c_str());

	this->cfg_brightnessThreashold = this->config->get_uint((this->cfg_prefix + "threashold_brightness").c_str());
	this->cfg_darknessThreashold = this->config->get_uint((this->cfg_prefix + "threashold_black").c_str());
	this->cfg_laserVisibilityThreashold = this->config->get_int((this->cfg_prefix + "threashold_laser_visibility").c_str());
	this->cfg_lightDistanceAllowedBetweenFrames = this->config->get_float((this->cfg_prefix + "light_distance_allowed_betwen_frames").c_str());
	this->cfg_lightOutOfRangeThrashold = this->config->get_int((this->cfg_prefix + "light_out_of_range_thrashold").c_str());

	this->cfg_lightSizeHeight = this->config->get_float((this->cfg_prefix + "light_size_height").c_str());
	this->cfg_lightSizeWidth = this->config->get_float((this->cfg_prefix + "light_size_width").c_str());

	this->cfg_detectionCycleTime = this->config->get_int((this->cfg_prefix + "detection_cycle_time").c_str());
	this->cfg_desiredLoopTime = this->config->get_float("/fawkes/mainapp/desired_loop_time") / 1000000;
	this->detectionCycleTimeFrames = cfg_detectionCycleTime / cfg_desiredLoopTime;

	std::string shmID = this->config->get_string((this->cfg_prefix + "shm_image_id").c_str());

	this->camera = vision_master->register_for_camera(this->cfg_camera.c_str(), this);

	this->img_width = this->camera->pixel_width();
	this->img_height = this->camera->pixel_height();

	this->cspaceFrom = this->camera->colorspace();

	this->scanline = new firevision::ScanlineGrid( this->img_width, this->img_height, 1, 1 );
	this->colorModel = new firevision::ColorModelBrightness(this->cfg_brightnessThreashold);

	this->classifierWhite = new firevision::SimpleColorClassifier(
			this->scanline,														//scanmodel
			this->colorModel,													//colorModel
			30,																	//num_min_points
			0,																	//box_extend
			false,																//upward
			2,																	//neighberhoud_min_match
			0,																	//grow_by
			firevision::C_WHITE													//color
			);

	this->colorModelBlack = new firevision::ColorModelDarkness(this->cfg_darknessThreashold);
	this->classifierBlack = new firevision::SimpleColorClassifier(
			this->scanline,														//scanmodel
			this->colorModelBlack,													//colorModel
			30,																	//num_min_points
			0,																	//box_extend
			false,																//upward
			2,																	//neighberhoud_min_match
			0,																	//grow_by
			firevision::C_BLACK													//color
			);

	// Create a ringbuffer with the size of the configured frame count
	this->historyBuffer = new boost::circular_buffer<lightSignal>(this->detectionCycleTimeFrames);
	//this->historyBuffer = new std::deque<lightSignal>();

	// SHM image buffer
	this->shmBufferYCbCr = new firevision::SharedMemoryImageBuffer(
			shmID.c_str(),
			this->cspaceTo,
			this->img_width,
			this->img_height
			);
	if (!shmBufferYCbCr->is_valid()) {
		throw fawkes::Exception("Shared memory segment not valid");
	}
	this->shmBufferYCbCr->set_frame_id(this->cfg_frame.c_str());

	this->bufferYCbCr = this->shmBufferYCbCr->buffer();

	//open interfaces
	this->nearestMaschineIF = blackboard->open_for_reading<fawkes::Position3DInterface>(
			this->config->get_string((this->cfg_prefix + "light_position_if").c_str()).c_str());

	this->lightStateIF = blackboard->open_for_writing<fawkes::RobotinoLightInterface>(
			this->config->get_string((this->cfg_prefix + "light_state_if").c_str()).c_str());

	//ROIs
	this->drawer = new firevision::FilterROIDraw();

	this->resetLightInterface();

	logger->log_debug(name(), "Plugin-light: end of init()");
}

void
PluginLightThread::finalize()													//TODO check if everthing gets deleted
{
	logger->log_debug(name(), "Plugin-light: start to free memory");

	vision_master->unregister_thread(this);

	delete this->camera;
	delete this->scanline;
	delete this->colorModel;
	delete this->classifierWhite;
	delete this->shmBufferYCbCr;

	blackboard->close(this->nearestMaschineIF);
	blackboard->close(this->lightStateIF);

	logger->log_info(name(), "Plugin-light: ends");
}

void
PluginLightThread::loop()
{
	bool contiueToPictureProcess = false;

	//read laser if
	this->nearestMaschineIF->read();
	int clusterVisibilityHistory = 5;// this->nearestMaschineIF->visibility_history();

	fawkes::cart_coord_3d_t lightPosition;
	lightPosition.x = 1; //this->nearestMaschineIF->translation(0);
	lightPosition.y = .04; //this->nearestMaschineIF->translation(1);
	lightPosition.z = this->nearestMaschineIF->translation(2);
	fawkes::polar_coord_2d_t lightPositionPolar;

	if ( clusterVisibilityHistory > this->cfg_laserVisibilityThreashold ) {
		if( clusterVisibilityHistory > 0 ){
			lightPositionPolar = this->transformCoordinateSystem(lightPosition, this->nearestMaschineIF->frame(), this->cfg_frame);

			if( this->isLightInViewarea(lightPositionPolar) ){
				this->lightOutOfRangeCounter = 0;
				contiueToPictureProcess = true;
			}
			else{
				if ( this->lightOutOfRangeCounter < this->cfg_lightOutOfRangeThrashold ) {
					if ( ! this->historyBuffer->empty()) {
						lightPositionPolar = this->historyBuffer->front().nearestMaschine_pos;

						contiueToPictureProcess = true;
					} else {
						this->resetLightInterface("light is out of range and buffer is empty");
					}
				} else {
					this->resetLightInterface("light is to often out of range for camera");
					this->resetLocalHistory();
				}
			}
		} else if ( ! this->historyBuffer->empty()) {
			lightPositionPolar = this->historyBuffer->front().nearestMaschine_pos;
			contiueToPictureProcess = true;
		} else {
			this->resetLightInterface("laser visibility is < 0 but buffer is empty");
		}
	} else{
		this->resetLightInterface("laser visibility lower than threashold");
		this->resetLocalHistory();
	}

	if ( contiueToPictureProcess ) {
		PluginLightThread::lightROIs lightROIs = this->calculateLightPos(lightPositionPolar);
		this->takePicture(lightROIs);

		lightROIs = this->correctLightRoisWithBlack(lightROIs);
		drawROIIntoBuffer(lightROIs.light);

		PluginLightThread::lightSignal lightSignalCurrentPicture = this->detectLightInCurrentPicture(lightROIs);
		lightSignalCurrentPicture.nearestMaschine_pos = lightPositionPolar;
		lightSignalCurrentPicture.nearestMaschine_history = clusterVisibilityHistory;
		this->historyBuffer->push_front(lightSignalCurrentPicture);

		this->processHistoryBuffer();
	}
}

bool
PluginLightThread::isLightInViewarea(fawkes::polar_coord_2d_t light)
{
	float cameraAngleDetectionArea = ( this->cfg_cameraFactorHorizontal / 2 ) - 0.1;

	if ( std::abs(light.phi) >= cameraAngleDetectionArea ) {
		return false;
	} else {
		return true;
	}
}

void
PluginLightThread::takePicture(PluginLightThread::lightROIs lightROIs) {

	camera->capture();
	firevision::convert(this->cspaceFrom,
			this->cspaceTo,
			this->camera->buffer(),
			this->bufferYCbCr,
			this->img_width,
			this->img_height);
	this->camera->dispose_buffer();

	//draw expected light in buffer
	if (this->cfg_paintROIsActivated) {
		this->drawROIIntoBuffer(lightROIs.light);
		this->drawROIIntoBuffer(lightROIs.red);
		this->drawROIIntoBuffer(lightROIs.yellow);
		this->drawROIIntoBuffer(lightROIs.green);
	}
}

firevision::ROI PluginLightThread::getBiggestRoi( std::list<firevision::ROI>* roiList) {
	firevision::ROI* biggestRoi = new firevision::ROI();

	for (std::list<firevision::ROI>::iterator it = roiList->begin();
			it != roiList->end(); ++it) {
		if (biggestRoi == NULL || biggestRoi->width * biggestRoi->height
				< (*it).width * (*it).height) {
			biggestRoi = &(*it);
		}
	}
	return biggestRoi;
}

PluginLightThread::lightROIs
PluginLightThread::correctLightRoisWithBlack(PluginLightThread::lightROIs expectedLight){
	// Look in area around the red light for the back top of the light

	firevision::ROI* top = new firevision::ROI(expectedLight.light);
	top->start.x = expectedLight.light.start.x - expectedLight.light.width;
	top->start.y = expectedLight.light.start.y - expectedLight.light.width;
	top->width =  expectedLight.light.width * 3;
	top->height = 2*expectedLight.light.width;

	this->drawROIIntoBuffer(top);

	// Look in the area around the green light for the black bottem
//	firevision::ROI* bottem = new firevision::ROI(top);
//	bottem->start.y = bottem->start.y + bottem->height;



	// Shrink or reposition the rois if sucessfully

//	int minStartX = 0;
//	int minStartY = 0;
//	int maxStartX = 0;
//	int maxStartY = 0;
//
//	std::list<firevision::ROI>* topBlack = this->classifyInRoi( top 	,this->classifierBlack);
//	for (std::list<firevision::ROI>::iterator it = topBlack->begin(); it != topBlack->end(); ++it) {
//		if(minStartX > (*it).start.x){
//			minStartX = (*it).start.x;
//		}
//
//		if(minStartY > (*it).start.y){
//			minStartY = (*it).start.y;
//		}
//
//		if(maxStartX < (*it).start.x){
//					minStartX = (*it).start.x;
//		}
//
//		if(maxStartY < (*it).start.y){
//			maxStartY = (*it).start.y;
//		}
//
//	}
	std::list<firevision::ROI>* topBlackList = this->classifyInRoi( top ,this->classifierBlack);
	if(!topBlackList->empty()){
		firevision::ROI topBiggestRoi = getBiggestRoi(topBlackList);
		this->drawROIIntoBuffer(topBiggestRoi);
		if (this->cfg_debugMessagesActivated) {
				logger->log_debug(name(), "BiggestRoi: X: %u Y: %u Height: %u Width: %u",topBiggestRoi.start.x , topBiggestRoi.start.y, topBiggestRoi.height, topBiggestRoi.width);
		}

		int TopDiffX = 0;
		int TopDiffY = 0;
		int TopDiffWidth = 0;


//			this->drawROIIntoBuffer(topBiggestRoi);
		TopDiffX = expectedLight.light.start.x - topBiggestRoi.start.x;
		TopDiffY = expectedLight.light.start.y - topBiggestRoi.start.y - topBiggestRoi.height;
		TopDiffWidth = expectedLight.light.width - topBiggestRoi.width;

		expectedLight.light.start.x = expectedLight.light.start.x - TopDiffX;
		expectedLight.light.start.y = expectedLight.light.start.y - TopDiffY;
		expectedLight.light.width = expectedLight.light.width - TopDiffWidth;

		expectedLight.red.start.x = expectedLight.red.start.x - TopDiffX;
		expectedLight.red.start.y = expectedLight.red.start.y - TopDiffY;
		expectedLight.red.width = expectedLight.light.width;

		expectedLight.yellow.start.x = expectedLight.yellow.start.x - TopDiffX;
		expectedLight.yellow.start.y = expectedLight.yellow.start.y - TopDiffY;
		expectedLight.yellow.width = expectedLight.light.width;

		expectedLight.green.start.x = expectedLight.green.start.x - TopDiffX;
		expectedLight.green.start.y = expectedLight.green.start.y - TopDiffY;
		expectedLight.green.width = expectedLight.light.width;

		drawROIIntoBuffer(expectedLight.red);
		drawROIIntoBuffer(expectedLight.yellow);
		drawROIIntoBuffer(expectedLight.green);

//
//		this->drawROIIntoBuffer(expectedLight.light);
	}else{
		logger->log_debug(name(), "No black roi found");
	}


//
//
//
//	//std::list<firevision::ROI>* bottemBlackList = this->classifyInRoi( bottem	,this->classifierBlack);
//	//firevision::ROI* bottemBiggestRoi = getBiggestRoi(bottemBlackList);



//	int BottemDiffX = 0;
//	int BottemDiffY = 0;
//	if( bottemBiggestRoi != NULL) {
//
//	}
	return expectedLight;
}

PluginLightThread::lightSignal
PluginLightThread::detectLightInCurrentPicture(PluginLightThread::lightROIs lightROIs)
{
	PluginLightThread::lightSignal lightSignal;

	lightSignal.red = this->signalLightCurrentPicture(lightROIs.red);
	lightSignal.yellow = this->signalLightCurrentPicture(lightROIs.yellow);
	lightSignal.green = this->signalLightCurrentPicture(lightROIs.green);

	return lightSignal;
}

void
PluginLightThread::processHistoryBuffer()
{
	if ( this->historyBuffer->full() ) {
		PluginLightThread::lightSignal lighSignal;

		if ( this->lightFromHistoryBuffer(lighSignal) ) {
			if ( lighSignal.red != fawkes::RobotinoLightInterface::UNKNOWN
			  && lighSignal.yellow != fawkes::RobotinoLightInterface::UNKNOWN
			  && lighSignal.green != fawkes::RobotinoLightInterface::UNKNOWN ) {

				this->writeLightInterface(lighSignal, true);

			} else {
				this->resetLightInterface("light couldn't detected");
			}
		} else {
			this->resetLightInterface("cluster jumped");
		}
	} else {
		this->writeLightInterface(this->historyBuffer->front(), false);
	}
}

PluginLightThread::~PluginLightThread()
{
	// TODO Auto-generated destructor stub
}

fawkes::polar_coord_2d_t
PluginLightThread::transformCoordinateSystem(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to)
{
	fawkes::polar_coord_2d_t polErrorReturnValue;
	this->cartToPol(polErrorReturnValue, cartFrom.x, cartFrom.y);

	bool world_frame_exists = tf_listener->frame_exists(from);
	bool robot_frame_exists = tf_listener->frame_exists(to);

	if (! world_frame_exists || ! robot_frame_exists) {
		logger->log_warn(name(), "Frame missing: %s %s   %s %s",
						 from.c_str(), world_frame_exists ? "exists" : "missing",
						 to.c_str(), robot_frame_exists ? "exists" : "missing");
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

		fawkes::tf::Vector3 v   = transform.getOrigin();

		fawkes::polar_coord_2d_t polTo;
		float toX, toY;

		toX = cartFrom.x + v.getX();
		toY = cartFrom.y + v.getY();
		this->cartToPol(polTo, toX, toY);

		if (this->cfg_debugMessagesActivated) {
			logger->log_debug(name(), "From: %s X: %f Y: %f", from.c_str(), cartFrom.x, cartFrom.y);
			logger->log_debug(name(), "To  : %s X: %f Y: %f", to.c_str(), toX, toY);
		}
		return polTo;
	}

	return polErrorReturnValue;
}

void PluginLightThread::cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y) {
	pol.phi = atan2f(y, x);
	pol.r = sqrtf(x * x + y * y);

	if (this->cfg_debugMessagesActivated) {
		logger->log_debug(name(), "x: %f; y: %f", x, y);
		logger->log_debug(name(), "Calculated r: %f; phi: %f", pol.r, pol.phi);
	}
}

void PluginLightThread::polToCart(float &x, float &y,fawkes::polar_coord_2d_t pol){
	x = pol.r * std::cos(pol.phi);
	y = pol.r * std::sin(pol.phi);
}

void
PluginLightThread::drawROIIntoBuffer(firevision::ROI roi, firevision::FilterROIDraw::border_style_t borderStyle)
{
	this->drawer->set_src_buffer(this->bufferYCbCr, firevision::ROI::full_image(this->img_width, this->img_height), 0);
	this->drawer->set_dst_buffer(this->bufferYCbCr, &roi);
	this->drawer->set_style(borderStyle);
	this->drawer->apply();

	if (this->cfg_debugMessagesActivated) {
		logger->log_debug(name(), "Plugin-light: drawed element in buffer");
	}
}

PluginLightThread::lightROIs
PluginLightThread::calculateLightPos(fawkes::polar_coord_2d_t lightPos)
{
	int expectedLightSizeWidth = this->img_width / (lightPos.r * this->cfg_cameraFactorHorizontal) * this->cfg_lightSizeWidth;
	int expectedLightSizeHeigth = this->img_height / (lightPos.r * this->cfg_cameraFactorVertical) * this->cfg_lightSizeHeight;	//TODO überprüfe welche einheit das Position3D interface nutzt, wenn es Meter sind, dann ist alles ok wenn es cm sind dann muss hier noch mit 100 Multiliziert werden

	float pixelPerRadHorizonal = this->img_width / this->cfg_cameraFactorHorizontal;

	int startX = this->img_width / 2											//picture center
				- lightPos.phi * pixelPerRadHorizonal							//move to the light
				- expectedLightSizeWidth / 2									//light center to light top cornor
				- this->cfg_cameraOffsetHorizontalRad * pixelPerRadHorizonal;	//angle of camera to robotor
				//TODO suche richtige werte der Kamera

	int startY = this->img_height / 2											//picture center
				- expectedLightSizeHeigth / 2									//light center to light cornor
				+ this->cfg_cameraOffsetVertical;								//error of picture position to light
				//TODO suche richtige werte der Kamera

	PluginLightThread::lightROIs lightROIs;

	//light ROI size
	lightROIs.light.image_height = this->img_height;
	lightROIs.light.image_width = this->img_width;
	lightROIs.light.height = expectedLightSizeHeigth;
	lightROIs.light.width = expectedLightSizeWidth;
	lightROIs.light.start.x = startX;
	lightROIs.light.start.y = startY;

	//Signale ROIs

	lightROIs.red = lightROIs.light;
	lightROIs.yellow = lightROIs.light;
	lightROIs.green = lightROIs.light;

	float roiHeight = ((float)lightROIs.light.height) / 9;

	lightROIs.red.height = roiHeight;
	lightROIs.red.start.y += roiHeight;											//Middle of the top thirds

	lightROIs.yellow.height = roiHeight;
	lightROIs.yellow.start.y += roiHeight * 4;									//Middle of the middle thirds

	lightROIs.green.height = roiHeight;
	lightROIs.green.start.y += roiHeight * 7;									//Middle of the bottom thirds

	return lightROIs;
}

void
PluginLightThread::writeLightInterface(PluginLightThread::lightSignal lightSignal, bool ready) {
	this->lightStateIF->read();
	int vis = this->lightStateIF->visibility_history();

	if (vis < 0) {
		vis = 1;
	} else {
		vis++;
	}
	this->lightStateIF->set_visibility_history(vis);

	this->lightStateIF->set_red(lightSignal.red);
	this->lightStateIF->set_yellow(lightSignal.yellow);
	this->lightStateIF->set_green(lightSignal.green);

	this->lightStateIF->set_ready(ready);

	this->lightStateIF->write();
}

//void
//PluginLightThread::updateLocalHistory(PluginLightThread::lightSignal lightSignalCurrent)
//{
//	this->historyBuffer->push_front(lightSignalCurrent);
//
//	if ( this->historyBuffer->full() ) {
//		this->writeLightInterface();
//	}
//
//}

void
PluginLightThread::resetLocalHistory() {
	this->historyBuffer->clear();
}

void
PluginLightThread::resetLightInterface(std::string message)
{
	this->lightStateIF->read();
	int vis = this->lightStateIF->visibility_history();

	if ( vis < 0 ) {
		vis--;
	} else {
		vis = -1;
	}
	this->lightStateIF->set_visibility_history(vis);

	this->lightStateIF->set_ready(false);
	this->lightStateIF->write();

	if (this->cfg_debugMessagesActivated) {
			logger->log_debug(name(), "Plugin-light: Resetting interface, %s",message.c_str());
	}
}

std::list<firevision::ROI>*
PluginLightThread::classifyInRoi(firevision::ROI searchArea, firevision::Classifier *classifier)
{
	this->scanline->reset();
	this->scanline->set_roi(&searchArea);

	classifier->set_src_buffer(this->bufferYCbCr, this->img_width, this->img_height);

	std::list<firevision::ROI> *ROIs = classifier->classify();

	return ROIs;
}

fawkes::RobotinoLightInterface::LightState
PluginLightThread::signalLightCurrentPicture(firevision::ROI signal)
{
	this->scanline->reset();
	this->scanline->set_roi(&signal);

	this->classifierWhite->set_src_buffer(this->bufferYCbCr, this->img_width, this->img_height);

	std::list<firevision::ROI> *ROIs = this->classifierWhite->classify();

	bool isOn = ! ROIs->empty();

	if (this->cfg_debugMessagesActivated) {
		int countedROIs = (int)ROIs->size();
		logger->log_info(name(), "Detect: %i", countedROIs);

		for (int i = 0; i < countedROIs; ++i) {
			this->drawROIIntoBuffer(ROIs->front());
			ROIs->pop_front();
		}
	}

	if (isOn) {
		return fawkes::RobotinoLightInterface::ON;
	} else {
		return fawkes::RobotinoLightInterface::OFF;
	}
}

bool
PluginLightThread::lightFromHistoryBuffer(PluginLightThread::lightSignal &lighSignal){

	int red = 0;
	int yellow = 0;
	int green = 0;
 // todo buffer von anfang nach ende durchlaufen
	PluginLightThread::lightSignal previousLight;
	for(boost::circular_buffer<PluginLightThread::lightSignal>::iterator it = this->historyBuffer->begin(); it != this->historyBuffer->end(); ++it){
		if(it != this->historyBuffer->begin()) {
			if( ! isValidSuccessor(*it,previousLight)){
				return false;
			}
		}
		if(it->red == fawkes::RobotinoLightInterface::ON){
			red++;
		}

		if(it->yellow == fawkes::RobotinoLightInterface::ON){
			yellow++;
		}

		if(it->green == fawkes::RobotinoLightInterface::ON){
			green++;
		}

		previousLight = *it;
	}

	lighSignal.red = signalLightWithHistory(red);
	lighSignal.yellow = signalLightWithHistory(yellow);
	lighSignal.green = signalLightWithHistory(green);

	return true;
}

bool
PluginLightThread::isValidSuccessor(lightSignal previous,lightSignal current){

	float px,py;
	float cx,cy;
	float dx,dy;

	polToCart(px,py,previous.nearestMaschine_pos);
	polToCart(cx,cy,current.nearestMaschine_pos);
	dx = cx - px;
	dy = cy - py;

	if(std::sqrt( (dx * dx) + (dy * dy)) < this->cfg_lightDistanceAllowedBetweenFrames ){
		return true;
	}
	return false;

}

fawkes::RobotinoLightInterface::LightState
PluginLightThread::signalLightWithHistory(int lightHistory)
{
	int visibilityHistory = this->historyBuffer->size();

	if ( lightHistory > ( visibilityHistory - this->cfg_lightNumberOfWrongDetections ) ) {
		return fawkes::RobotinoLightInterface::ON;

	} else if ( lightHistory > visibilityHistory / 2 - this->cfg_lightNumberOfWrongDetections
			&&  lightHistory < visibilityHistory / 2 + this->cfg_lightNumberOfWrongDetections ) {
		return fawkes::RobotinoLightInterface::BLINKING; // todo check blinking max allowed errors

	} else if ( lightHistory < this->cfg_lightNumberOfWrongDetections ) {
		return fawkes::RobotinoLightInterface::OFF;
	} else {
		return fawkes::RobotinoLightInterface::UNKNOWN;
	}
}












