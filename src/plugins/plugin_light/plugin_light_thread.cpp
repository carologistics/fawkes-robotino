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
	this->cfg_prefix_ = "";
	this->cfg_camera_ = "";
	this->cfg_frame_ = "";

	this->cfg_threashold_brightness_ = 0;
	this->cfg_threashold_roiMaxSize_ = 0;

	this->img_width_ = 0;
	this->img_height_ = 0;

	this->camOffsetTop = 0;
	this->camOffsetBottom = 0;

	this->buffer_YCbCr = NULL;

	this->cam_ = NULL;
	this->scanline_ = NULL;
	this->colorModel_ = NULL;
	this->classifier_light_ = NULL;
	this->shm_buffer_YCbCr = NULL;
}

void
PluginLightThread::init()
{
	logger->log_info(name(), "Plugin-light: starts up");

	this->cfg_prefix_ = "/plugins/plugin_light/";
	this->cfg_camera_ = this->config->get_string((this->cfg_prefix_ + "camera").c_str());
	this->cfg_frame_  = this->config->get_string((this->cfg_prefix_ + "frame").c_str());

	this->cfg_threashold_brightness_ = this->config->get_uint((this->cfg_prefix_ + "threashold_brightness").c_str());
	this->cfg_threashold_roiMaxSize_ = this->config->get_uint((this->cfg_prefix_ + "threashold_roiMaxSize").c_str());

	this->camOffsetTop = this->config->get_uint((this->cfg_prefix_ + "camera_offset_top").c_str());
	this->camOffsetBottom = this->config->get_uint((this->cfg_prefix_ + "camera_offset_bottom").c_str());

	const char* file = this->config->get_string((this->cfg_prefix_ + "imgcam").c_str()).c_str();
	logger->log_info(name(), file);
	this->cam_ = new firevision::FileLoader(file);
	this->cam_->open();
//	this->cam_ = vision_master->register_for_camera(this->cfg_camera_.c_str(), this);

	this->img_width_ = this->cam_->pixel_width();
	this->img_height_ = this->cam_->pixel_height();

	this->cspace_from_ = this->cam_->colorspace();
	this->cspace_to_ = firevision::YUV422_PLANAR;

	this->scanline_ = new firevision::ScanlineGrid( this->img_width_, this->img_height_, 1, 1 );
	this->colorModel_ = new firevision::ColorModelBrightness(this->cfg_threashold_brightness_);

	this->classifier_light_ = new firevision::SimpleColorClassifier(
			this->scanline_,													//scanmodel
			this->colorModel_,													//colorModel
			1,																	//num_min_points
			2,																	//box_extend
			false,																//upward
			2,																	//neighberhoud_min_match
			0,																	//grow_by
			firevision::C_WHITE													//color
			);

	// SHM image buffer
//	unsigned int img_heightMinusOffset = this->img_height_						//buffer height is ori height - top and bottom offset
//											 - this->camOffsetTop
//											 - this->camOffsetBottom;


	this->shm_buffer_YCbCr = new firevision::SharedMemoryImageBuffer(
			"Light-cam YUV",
			this->cspace_to_,
			this->img_width_,
			this->img_height_
//			img_heightMinusOffset
			);
	if (!shm_buffer_YCbCr->is_valid()) {
		throw fawkes::Exception("Shared memory segment not valid");
	}
	this->shm_buffer_YCbCr->set_frame_id(this->cfg_frame_.c_str());

	this->buffer_YCbCr = this->shm_buffer_YCbCr->buffer();

	logger->log_debug(name(), "Plugin-light: end of init()");
}

void
PluginLightThread::finalize()
{
	logger->log_debug(name(), "Plugin-light: start to free memory");

	delete this->cam_;
	delete this->scanline_;
	delete this->colorModel_;
	delete this->classifier_light_;
	delete this->shm_buffer_YCbCr;

	logger->log_info(name(), "Plugin-light: ends");
}

void
PluginLightThread::loop()
{
	cam_->capture();

	//copy cam buffer to local buffer and remove picture parts at the top and bottom
//	unsigned char* camBufferStartPosition = this->cam_->buffer()				//startpossition of buffer is ori position + top offset
//											+ ( this->camOffsetTop * this->img_width_ ) ;
//	unsigned int img_heightMinusOffset = this->img_height_						//buffer height is ori height - top and bottom offset
//										 - this->camOffsetTop
//										 - this->camOffsetBottom;

	std::memcpy(this->shm_buffer_YCbCr->buffer(), this->cam_->buffer(), this->cam_->buffer_size());

//	firevision::convert(
//			this->cspace_from_,
//			this->cspace_to_,
//			camBufferStartPosition,
////			this->cam_->buffer(),
//			this->buffer_YCbCr,
//			this->img_width_,
//			img_heightMinusOffset
////			this->img_height_
//			);

	//search for ROIs
//	this->getROIs(camBufferStartPosition, this->img_width_, img_heightMinusOffset);

	//draw ROIs in buffer

	//do stuff with rois

	this->cam_->dispose_buffer();
}

PluginLightThread::~PluginLightThread()
{
	// TODO Auto-generated destructor stub
}

std::list<firevision::ROI>*
PluginLightThread::getROIs(unsigned char *buffer, unsigned int imgWidth, unsigned int imgHeight_)
{
	std::list<firevision::ROI>* roiList = new std::list<firevision::ROI>();

	scanline_->reset();
	this->classifier_light_->set_src_buffer(buffer, imgWidth, imgHeight_);
	this->classifier_light_->classify();

	std::list<firevision::ROI> *roiListSmall = new std::list<firevision::ROI>();
	firevision::ROI *tmpRoi = NULL;

	while ( ! roiList->empty() ) {
		tmpRoi = new firevision::ROI(roiList->front());

		if(tmpRoi->get_height() * tmpRoi->get_width() > this->cfg_threashold_roiMaxSize_) {

		} else {
			roiListSmall->push_back(*tmpRoi);
		}
		roiList->pop_front();
	}
	delete roiList;
	roiList = roiListSmall;

	return roiList;
}


















