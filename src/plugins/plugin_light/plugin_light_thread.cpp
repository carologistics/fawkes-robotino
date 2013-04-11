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

	this->cfg_cameraAngleHorizontal = 0;
	this->cfg_cameraAngleVertical = 0;

	this->cfg_lightSizeHeight = 0;
	this->cfg_lightSizeWidth = 0;

	this->cfg_threasholdBrightness = 0;

	this->img_width = 0;
	this->img_height = 0;
	this->img_heightMinusOffset = 0;

	this->cfg_cameraOffsetTop = 0;
	this->cfg_cameraOffsetBottom = 0;

	this->bufferYCbCr = NULL;

	this->cspaceFrom = firevision::YUV422_PLANAR;
	this->cspaceTo = firevision::YUV422_PLANAR;

	this->camera = NULL;
	this->scanline = NULL;
	this->colorModel = NULL;
	this->classifierLight = NULL;
	this->shmBufferYCbCr = NULL;

	this->lightPositionLasterIF = NULL;

	this->cfg_debugMessages = false;
}

void
PluginLightThread::init()
{
	logger->log_info(name(), "Plugin-light: starts up");

	this->cfg_prefix = "/plugins/plugin_light/";

	this->cfg_frame  = this->config->get_string((this->cfg_prefix + "frame").c_str());

	this->cfg_camera = this->config->get_string((this->cfg_prefix + "camera").c_str());
	this->cfg_cameraOffsetTop = this->config->get_uint((this->cfg_prefix + "camera_offset_top").c_str());
	this->cfg_cameraOffsetBottom = this->config->get_uint((this->cfg_prefix + "camera_offset_bottom").c_str());
	this->cfg_cameraAngleHorizontal = this->config->get_float((this->cfg_prefix + "camera_angle_horizontal").c_str());
	this->cfg_cameraAngleVertical = this->config->get_float((this->cfg_prefix + "camera_angle_vertical").c_str());

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

	//for later remove of unused parts of the picture
	this->img_heightMinusOffset = this->img_height								//buffer height is ori height - top and bottom offset
									 - this->cfg_cameraOffsetTop
									 - this->cfg_cameraOffsetBottom;

	// SHM image buffer
	this->shmBufferYCbCr = new firevision::SharedMemoryImageBuffer(
			shmID.c_str(),
			this->cspaceTo,
			this->img_width,
//			this->img_height_
			this->img_heightMinusOffset
			);
	if (!shmBufferYCbCr->is_valid()) {
		throw fawkes::Exception("Shared memory segment not valid");
	}
	this->shmBufferYCbCr->set_frame_id(this->cfg_frame.c_str());

	this->bufferYCbCr = this->shmBufferYCbCr->buffer();

	//open interfaces
	this->lightPositionLasterIF = blackboard->open_for_reading<fawkes::PolarPosition2DInterface>(
			this->config->get_string((this->cfg_prefix + "light_position_if").c_str()).c_str());

	logger->log_debug(name(), "Plugin-light: end of init()");

//	if (this->cfg_debugMessages) {
//		logger->log_info(name(), "");
//	}
}

unsigned char*
PluginLightThread::calculatePositionInCamBuffer()
{
	return this->camera->buffer()													//startpossition of buffer is ori position + top offset
				+ ( this->cfg_cameraOffsetTop * this->img_width ) ;
}

void
PluginLightThread::finalize()
{
	logger->log_debug(name(), "Plugin-light: start to free memory");

	delete this->camera;
	delete this->scanline;
	delete this->colorModel;
	delete this->classifierLight;
	delete this->shmBufferYCbCr;

//	blackboard->close(this->lightPositionLaster_if);							//TODO finde fehler

	logger->log_info(name(), "Plugin-light: ends");
}

void
PluginLightThread::loop()
{
	camera->capture();

	//copy cam buffer to local buffer and remove picture parts at the top and bottom
	unsigned char* camBufferStartPosition = this->calculatePositionInCamBuffer();

	firevision::convert(
			this->cspaceFrom,
			this->cspaceTo,
			camBufferStartPosition,
//			this->cam_->buffer(),
			this->bufferYCbCr,
			this->img_width,
			this->img_heightMinusOffset
//			this->img_height_
			);
	this->camera->dispose_buffer();

	//read laser if
	this->lightPositionLasterIF->read();
	fawkes::polar_coord_2d_t lightPosition;

	lightPosition.phi = this->lightPositionLasterIF->angle();
	lightPosition.r = this->lightPositionLasterIF->distance();

	//transform coorodinate-system from laser -> camera
	//from this->lightPositionLasterIF->frame();
	//to this->cfg_frame;
	//beispiel: fawkes-robotino/fawkes/src/plugins/examples/tf_example

	//draw expected camera in buffer
	this->drawLightIntoBuffer(lightPosition);

	//search for ROIs
	std::list<firevision::ROI>* ROIs =
	this->getROIs(
			camBufferStartPosition,
			this->img_width,
			this->img_heightMinusOffset
			);

	//draw ROIs in buffer

	//do stuff with rois


//	delete ROIs;
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

	return roiList;
}

void
PluginLightThread::drawLightIntoBuffer(fawkes::polar_coord_2d_t positionOfLight)
{
//	positionOfLight.phi
//	positionOfLight.r

	int expectedLightSizeHeigth = 60;//*this->cfg_lightSize_height;
	int expectedLightSizeWidth =  20;//*this->cfg_lightSize_width;

	float radPerPixelHorizonal = img_width_/cfg_cameraAngleHorizontal;
//	float radPerPixelVertical = img_width_/cfg_cameraAngleVertical;

	int startX = positionOfLight.phi*radPerPixelHorizonal + (img_width_-expectedLightSizeWidth)/2;
	int startY = (img_heightMinusOffset-expectedLightSizeHeigth)/2;

	firevision::ROI *light = new firevision::ROI(startX,startY,expectedLightSizeWidth,expectedLightSizeHeigth,img_width_,img_heightMinusOffset);

	firevision::FilterROIDraw *drawer = new firevision::FilterROIDraw();
	drawer->set_src_buffer(this->buffer_YCbCr, firevision::ROI::full_image(img_width_, img_heightMinusOffset), 0);
	drawer->set_dst_buffer(this->buffer_YCbCr, light);
	drawer->set_style(firevision::FilterROIDraw::INVERTED);
	drawer->apply();

	logger->log_info(name(), "Plugin-light: drawed element in buffer");
}




















