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

	this->cfg_threasholdBrightness = 0;

	this->img_width = 0;
	this->img_height = 0;

	this->bufferYCbCr = NULL;

	this->cspaceFrom = firevision::YUV422_PLANAR;
	this->cspaceTo = firevision::YUV422_PLANAR;

	this->camera = NULL;
	this->scanline = NULL;
	this->colorModel = NULL;
	this->classifierLight = NULL;
	this->shmBufferYCbCr = NULL;

	this->lightPositionLasterIF = NULL;
	this->lightStateIF = NULL;

	this->drawer = NULL;

	this->cfg_debugMessages = false;
}

void
PluginLightThread::init()
{
	logger->log_info(name(), "Plugin-light: starts up");

	this->cfg_prefix = "/plugins/plugin_light/";

	this->cfg_frame  = this->config->get_string((this->cfg_prefix + "frame").c_str());

	this->cfg_camera = this->config->get_string((this->cfg_prefix + "camera").c_str());
	this->cfg_cameraFactorHorizontal = this->config->get_float((this->cfg_prefix + "camera_factor_horizontal").c_str());
	this->cfg_cameraFactorVertical = this->config->get_float((this->cfg_prefix + "camera_factor_vertical").c_str());

	this->cfg_cameraOffsetHorizontalRad = this->config->get_float((this->cfg_prefix + "camera_offset_horizontal_rad").c_str());
	this->cfg_cameraOffsetVertical = this->config->get_int((this->cfg_prefix + "camera_offset_vertical").c_str());

	this->cfg_debugMessages = this->config->get_bool((this->cfg_prefix + "show_debug_messages").c_str());

	this->cfg_threasholdBrightness = this->config->get_uint((this->cfg_prefix + "threashold_brightness").c_str());

	this->cfg_lightSizeHeight = this->config->get_float((this->cfg_prefix + "light_size_height").c_str());
	this->cfg_lightSizeWidth = this->config->get_float((this->cfg_prefix + "light_size_width").c_str());

	std::string shmID = this->config->get_string((this->cfg_prefix + "shm_image_id").c_str());

	this->camera = vision_master->register_for_camera(this->cfg_camera.c_str(), this);

	this->img_width = this->camera->pixel_width();
	this->img_height = this->camera->pixel_height();

	this->cspaceFrom = this->camera->colorspace();

	this->scanline = new firevision::ScanlineGrid( this->img_width, this->img_height, 1, 1 );
	this->colorModel = new firevision::ColorModelBrightness(this->cfg_threasholdBrightness);

	this->classifierLight = new firevision::SimpleColorClassifier(
			this->scanline,													//scanmodel
			this->colorModel,													//colorModel
			1,																	//num_min_points
			2,																	//box_extend
			false,																//upward
			2,																	//neighberhoud_min_match
			0,																	//grow_by
			firevision::C_WHITE													//color
			);

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
	this->lightPositionLasterIF = blackboard->open_for_reading<fawkes::Position3DInterface>(
			this->config->get_string((this->cfg_prefix + "light_position_if").c_str()).c_str());

	this->lightStateIF = blackboard->open_for_writing<fawkes::RobotinoLightInterface>(
			this->config->get_string((this->cfg_prefix + "light_state_if").c_str()).c_str());

	//ROIs
	this->drawer = new firevision::FilterROIDraw();

	logger->log_debug(name(), "Plugin-light: end of init()");

//	if (this->cfg_debugMessages) {
//		logger->log_info(name(), "");
//	}
}

void
PluginLightThread::finalize()													//TODO check if everthing gets deleted
{
	logger->log_debug(name(), "Plugin-light: start to free memory");

	delete this->camera;
	delete this->scanline;
	delete this->colorModel;
	delete this->classifierLight;
	delete this->shmBufferYCbCr;

	blackboard->close(this->lightPositionLasterIF);

	logger->log_info(name(), "Plugin-light: ends");
}

void
PluginLightThread::loop()
{
	//read laser if
	this->lightPositionLasterIF->read();
	fawkes::cart_coord_3d_t lightPosition;

	lightPosition.x = this->lightPositionLasterIF->translation(0);
	lightPosition.y = this->lightPositionLasterIF->translation(1);
	lightPosition.z = this->lightPositionLasterIF->translation(2);

	//transform coorodinate-system from laser -> camera
	std::string lightPosFrame = this->lightPositionLasterIF->frame();

	fawkes::polar_coord_2d_t lightPositionPolar = this->transformPolarCoord2D(lightPosition, lightPosFrame, this->cfg_frame);

	lightPositionPolar.r = 1;
	lightPositionPolar.phi = 0;
	PluginLightThread::lightROIs lightROIs = this->calculateLightPos(lightPositionPolar);

	logger->log_info(name(), "x: %u, y: %u, h: %u, w: %u", lightROIs.light.start.x, lightROIs.light.start.x, lightROIs.light.height, lightROIs.light.width);

	camera->capture();

	firevision::convert(
			this->cspaceFrom,
			this->cspaceTo,
			this->camera->buffer(),
			this->bufferYCbCr,
			this->img_width,
			this->img_height
			);
	this->camera->dispose_buffer();

	//draw expected light in buffer
	this->drawROIIntoBuffer(lightROIs.light);
//	this->drawROIIntoBuffer(lightROIs.red);
//	this->drawROIIntoBuffer(lightROIs.yellow);
//	this->drawROIIntoBuffer(lightROIs.green);

	//detect signals
	this->isSignalOn(lightROIs.red);
	this->isSignalOn(lightROIs.yellow);
	this->isSignalOn(lightROIs.green);

	//draw ROIs in buffer

	//do stuff with rois

	//write light state
	this->writeLightInterface(
			fawkes::RobotinoLightInterface::ON,
			fawkes::RobotinoLightInterface::BLINKING,
			fawkes::RobotinoLightInterface::OFF,
			true
			);

//	delete ROIs;
//	delete ROIsInLightArea;
}

PluginLightThread::~PluginLightThread()
{
	// TODO Auto-generated destructor stub
}

std::list<firevision::ROI>*
PluginLightThread::getROIs(unsigned char *buffer, unsigned int imgWidth, unsigned int imgHeight_)
{
	std::list<firevision::ROI>* roiList = new std::list<firevision::ROI>();

	//search for ROIs
	scanline->reset();
	this->classifierLight->set_src_buffer(buffer, imgWidth, imgHeight_);
	roiList = this->classifierLight->classify();

	return roiList;
}

std::list<firevision::ROI>*
PluginLightThread::removeUnimportantROIs(std::list<firevision::ROI>* ROIs, firevision::ROI light)
{
	//	//remove ROIs that are too big
	//	std::list<firevision::ROI> *roiListSmall = new std::list<firevision::ROI>();
	//	firevision::ROI *tmpRoi = NULL;
	//
	//	while ( ! roiList->empty() ) {
	//		tmpRoi = new firevision::ROI(roiList->front());
	//
	//		if(tmpRoi->get_height() * tmpRoi->get_width() > this->cfg_threashold_roiMaxSize_) {
	//
	//		} else {
	//			roiListSmall->push_back(*tmpRoi);
	//		}
	//		delete tmpRoi;
	//		roiList->pop_front();
	//	}
	//	delete roiList;
	//	roiList = roiListSmall;
	return ROIs;
}

fawkes::polar_coord_2d_t
PluginLightThread::transformPolarCoord2D(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to)
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

		if (this->cfg_debugMessages) {
			logger->log_info(name(), "From: %s X: %f Y: %f", from.c_str(), cartFrom.x, cartFrom.y);
			logger->log_info(name(), "To  : %s X: %f Y: %f", to.c_str(), toX, toY);
		}
		return polTo;
	}

	return polErrorReturnValue;
}

//void PluginLightThread::polToCart(fawkes::polar_coord_2d_t pol, float &x, float &y) {
//	x = pol.r * cos(pol.phi * M_PI / 180.0);
//	y = pol.r * sin(pol.phi * M_PI / 180.0);
//}

void PluginLightThread::cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y) {
	pol.phi = atan2f(y, x) * 180 / M_PI;
	pol.r = sqrtf(x * x + y * y);
}

void
PluginLightThread::drawROIIntoBuffer(firevision::ROI roi, firevision::FilterROIDraw::border_style_t borderStyle)
{
	this->drawer->set_src_buffer(this->bufferYCbCr, firevision::ROI::full_image(this->img_width, this->img_height), 0);
	this->drawer->set_dst_buffer(this->bufferYCbCr, &roi);
	this->drawer->set_style(borderStyle);
	this->drawer->apply();

	if (this->cfg_debugMessages) {
		logger->log_info(name(), "Plugin-light: drawed element in buffer");
	}
}

PluginLightThread::lightROIs
PluginLightThread::calculateLightPos(fawkes::polar_coord_2d_t lightPos)
{
	int expectedLightSizeHeigth = this->img_height / (lightPos.r * this->cfg_cameraFactorVertical) * this->cfg_lightSizeHeight;	//TODO überprüfe welche einheit das Position3D interface nutzt, wenn es Meter sind, dann ist alles ok wenn es cm sind dann muss hier noch mit 100 Multiliziert werden
	int expectedLightSizeWidth = this->img_width / (lightPos.r * this->cfg_cameraFactorHorizontal) * this->cfg_lightSizeWidth;

	float radPerPixelHorizonal = this->img_width / this->cfg_cameraFactorHorizontal;
//	float radPerPixelVertical = img_width/cfg_cameraAngleVertical;

	int startX = lightPos.phi * radPerPixelHorizonal + (this->img_width - expectedLightSizeWidth) / 2
							+ this->cfg_cameraOffsetHorizontalRad * lightPos.r * radPerPixelHorizonal;	//TODO suche richtige werte der Kamera
	int startY = (this->img_height - expectedLightSizeHeigth) / 2
							+ this->cfg_cameraOffsetVertical;											//TODO suche richtige werte der Kamera

	PluginLightThread::lightROIs lightROIs;

	//light ROI size
	lightROIs.light.image_height = this->img_height;
	lightROIs.light.image_width = this->img_width;
	lightROIs.light.height = expectedLightSizeHeigth;
	lightROIs.light.width = expectedLightSizeWidth;
	lightROIs.light.start.x = startX;
	lightROIs.light.start.y = startY;

	lightROIs.red = lightROIs.light;
	lightROIs.yellow = lightROIs.light;
	lightROIs.green = lightROIs.light;

	//Signale ROIs
	lightROIs.red.height /= 3;

	lightROIs.yellow.height /= 3;
	lightROIs.yellow.start.y += lightROIs.red.height;

	lightROIs.green.height /= 3;
	lightROIs.green.start.y += lightROIs.red.height + lightROIs.yellow.height;

	return lightROIs;
}

void
PluginLightThread::writeLightInterface(fawkes::RobotinoLightInterface::LightState red,
		 fawkes::RobotinoLightInterface::LightState yellow,
		 fawkes::RobotinoLightInterface::LightState green,
		 bool ready,
		 bool resetVisibilityHistory)
{
	if (resetVisibilityHistory) {
		this->lightStateIF->set_visibility_history(-1);
	} else {
		this->lightStateIF->read();
		int vis = this->lightStateIF->visibility_history();
		this->lightStateIF->set_visibility_history(vis + 1);
	}

	this->lightStateIF->set_green(green);
	this->lightStateIF->set_yellow(yellow);
	this->lightStateIF->set_red(red);

	this->lightStateIF->set_ready(ready);

	this->lightStateIF->write();
}

bool
PluginLightThread::isSignalOn(firevision::ROI signal)
{
	this->scanline->reset();
	this->scanline->set_roi(&signal);

	this->classifierLight->set_src_buffer(this->bufferYCbCr, this->img_width, this->img_height);

	std::list<firevision::ROI> *ROIs = this->classifierLight->classify();

	if (this->cfg_debugMessages) {
		int countedROIs = (int)ROIs->size();
		logger->log_info(name(), "Detect: %i", countedROIs);

		for (int i = 0; i < countedROIs; ++i) {
			this->drawROIIntoBuffer(ROIs->front());
			ROIs->pop_front();
		}
	}

	return true;
}














