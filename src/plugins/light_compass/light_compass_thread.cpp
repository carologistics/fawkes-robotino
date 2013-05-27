/***************************************************************************
 *  light_compass_thread.cpp - light thread
 *
 *  Created: Mi 23. Mai 17:44:14 CEST 2012
 *  Copyright  2012 Daniel Ewert 
 *
 ****************************************************************************/

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

#include "light_compass_thread.h"


#include <cmath>


using namespace fawkes;

/** @class LightCompassThread "light_compass_thread.h"
 * Introductional example thread which makes the robotino drive along a 8-shaped course
 * @author Daniel Ewert
 */

/** Constructor. */
LightCompassThread::LightCompassThread() :
	Thread("LightCompassThread", Thread::OPMODE_WAITFORWAKEUP), VisionAspect(
			VisionAspect::CYCLIC) {

	this->scanline_ = NULL;
	this->cm_ = NULL;
	this->mirror_ = NULL;
	this->rel_pos_ = NULL;
	this->classifier_white_ = NULL;
	this->colorModel_ = NULL;
	this->shm_buffer_YCbCr = NULL;

	this->cspace_to_ = firevision::YUV422_PLANAR;
}


void LightCompassThread::init()
{
	logger->log_info(name(), "Plugin light-compass starts up");

	this->cfg_prefix_        = "/plugins/light_compass/";
	this->cfg_camera_        = this->config->get_string((cfg_prefix_ + "camera").c_str());
	this->cfg_mirror_file_   = std::string(CONFDIR) + "/" +
							   this->config->get_string((cfg_prefix_ + "mirror_file").c_str());
	this->cfg_frame_         = this->config->get_string((cfg_prefix_ + "frame").c_str());

	this->cfg_mirror_height  = this->config->get_float((cfg_prefix_ + "mirror_height").c_str());
	this->cfg_robot_radius_  = this->config->get_float((cfg_prefix_ + "robot_radius").c_str());

	this->cfg_red_height_    = this->config->get_float((cfg_prefix_ + "red_height").c_str());
	this->cfg_orange_height_ = this->config->get_float((cfg_prefix_ + "orange_height").c_str());
	this->cfg_green_height_  = this->config->get_float((cfg_prefix_ + "green_height").c_str());

	this->cfg_useBulbFile_ = this->config->get_bool((cfg_prefix_ + "mirror_correction").c_str());;
	this->cfg_debugOutput_ = this->config->get_bool((cfg_prefix_ + "debug").c_str());;

	this->cfg_threashold_brightness_ = this->config->get_uint((cfg_prefix_ + "threashold_brightness").c_str());
	this->cfg_threashold_roiMaxSize_ = this->config->get_uint((cfg_prefix_ + "threashold_roiMaxSize").c_str());

	this->cfg_lightDistanceAllowedBetweenFrames = this->config->get_float((cfg_prefix_ + "light_distance_allowed_betwen_frames").c_str());

	this->lightif = blackboard->open_for_writing<Position3DInterface>("nearest_light_on");
	this->lightif->set_frame(this->cfg_frame_.c_str());
	this->lightif->set_visibility_history(-1);
	this->lightif->write();

	this->cam_ = vision_master->register_for_camera(this->cfg_camera_.c_str(), this);
	this->img_width_ = this->cam_->pixel_width();
	this->img_height_ = this->cam_->pixel_height();
	this->cspace_from_ = this->cam_->colorspace();

	// SHM image buffer
	this->shm_buffer_YCbCr = new firevision::SharedMemoryImageBuffer("lightCompas-YUV", this->cspace_to_,
			this->img_width_, this->img_height_);
	if (!shm_buffer_YCbCr->is_valid()) {
		throw Exception("Shared memory segment not valid");
	}
	this->shm_buffer_YCbCr->set_frame_id(this->cfg_frame_.c_str());
	this->buffer_YCbCr = shm_buffer_YCbCr->buffer();

	// SHM image buffer
	this->shm_buffer_filtered = new firevision::SharedMemoryImageBuffer("lightCompas-ROI", this->cspace_to_,
			this->img_width_, this->img_height_);
	if (!shm_buffer_filtered->is_valid()) {
		throw Exception("Shared memory segment not valid");
	}
	this->shm_buffer_filtered->set_frame_id(this->cfg_frame_.c_str());
	this->buffer_filtered = shm_buffer_filtered->buffer();

	// scanline_ model
	this->scanline_ = new firevision::ScanlineRadial(
			this->img_width_,
			this->img_height_,
			this->img_width_/2, //center width
			this->img_height_ /2, //center height
			2, //radius increment
			2, // step
			this->img_height_/2,//max radius
			100 // dead radius
			);

	this->colorModel_ = new firevision::ColorModelLuminance(this->cfg_threashold_brightness_);

	//										scanmodel, colorModel, num_min_points, box_extend, upward, neighberhoud_min_match, grow_by, color
	this->classifier_white_ = new firevision::SimpleColorClassifier(this->scanline_, this->colorModel_, 1, 2, false, 2, 0, firevision::C_WHITE);

	this->filterROIDraw = new firevision::FilterROIDraw();

	this->mirror_ = new firevision::Bulb(this->cfg_mirror_file_.c_str());

	logger->log_debug(name(), "Cam String = %s", this->cfg_camera_.c_str());
	logger->log_debug(name(), "Mirror     = %s", this->cfg_mirror_file_.c_str());
	logger->log_debug(name(), "Frame      = %s", this->cfg_frame_.c_str());

	logger->log_debug(name(), "Mirror H   = %f", this->cfg_mirror_height);
	logger->log_debug(name(), "Radius H   = %f", this->cfg_robot_radius_);
	logger->log_debug(name(), "RED    H   = %f", this->cfg_red_height_);
	logger->log_debug(name(), "ORANGE H   = %f", this->cfg_orange_height_);
	logger->log_debug(name(), "GREEN  H   = %f", this->cfg_green_height_);

	logger->log_debug(name(), "Brightness = %i", this->cfg_threashold_brightness_);
	logger->log_debug(name(), "ROI max S  = %i", this->cfg_threashold_roiMaxSize_);

	logger->log_info(name(), "Light Compass init done.");
}

bool LightCompassThread::prepare_finalize_user()
{
	return true;
}

void LightCompassThread::finalize()
{
	logger->log_debug(name(), "Unregistering from vision master");
	vision_master->unregister_thread(this);
	delete this->cam_;

	this->lightif->set_visibility_history(0);
	this->lightif->write();
	blackboard->close(this->lightif);
	delete shm_buffer_YCbCr;
	delete shm_buffer_filtered;
	delete scanline_;
	delete colorModel_;
	delete classifier_white_;
	delete filterROIDraw;
	delete mirror_;
}

void LightCompassThread::loop()
{
	logger->log_debug(name(),"light-compass: loop");
	//lese die Camera
	cam_->capture();
	//berechne in einen "guten" Farb-Raum um
	firevision::convert(cspace_from_, cspace_to_, cam_->buffer(), buffer_YCbCr, img_width_,img_height_);
	cam_->dispose_buffer();

	//Suche ROIs Licht PositionEN
	//unsigned int* lightPosInPicture = this->getBestROI(this->buffer_YCbCr);
	firevision::ROI* bestRoi = this->getBestROI(this->buffer_YCbCr);

	if( bestRoi != NULL ){
		//Zeichne ROIS in 2. Buffer
		firevision::convert(cspace_to_, cspace_to_, buffer_YCbCr, buffer_filtered, img_width_,img_height_);
		this->drawROIInBuffer(this->buffer_filtered, this->img_width_, this->img_height_, bestRoi);
		logger->log_debug(name(),"Position Pixel: %u, %u", bestRoi->start.x , bestRoi->start.y);

		//rechne verzerrung um
		fawkes::polar_coord_2d_t bestLightPosRelPolar = this->mirrorCorrectionOfROI(bestRoi);
		logger->log_debug(name(),"Position polar (mirror) r: %f phi: %f", bestLightPosRelPolar.r, bestLightPosRelPolar.phi);

		//rechnte cartesische coordinaten
		float bestPosCartX=0;
		float bestPosCartY=0;
		fawkes::polar2cart2d(bestLightPosRelPolar.phi, bestLightPosRelPolar.r, &bestPosCartX, &bestPosCartY);
		logger->log_debug(name(),"Position Cart: %f, %f Rotation: %f Radius: %f",bestPosCartX ,bestPosCartY ,bestLightPosRelPolar.phi ,bestLightPosRelPolar.r);

		//speicher in interface
		this->UpdateInterface((double)bestPosCartX,(double)bestPosCartY, &bestLightPosRelPolar);

	} else {
		this->ResetInterface();
	}

}

fawkes::polar_coord_2d_t LightCompassThread::mirrorCorrectionOfROI(firevision::ROI* roi)
{
	fawkes::polar_coord_2d_t roiPosPolar;
	roiPosPolar.r=0;
	roiPosPolar.phi=0;

	if ( roi != NULL ) {
		//umrechen
		unsigned int bestRoiCenterPosX = roi->start.x + roi->width / 2;
		unsigned int bestRoiCenterPosY = roi->start.y + roi->height / 2;

		if(cfg_useBulbFile_){
			roiPosPolar = this->mirror_->getWorldPointRelative(bestRoiCenterPosX, bestRoiCenterPosY);
		}
		else{
			fawkes::cart2polar2d(bestRoiCenterPosX,bestRoiCenterPosY,&roiPosPolar.phi,&roiPosPolar.r);
		}
		roiPosPolar = this->distanceCorrectionOfAmple(roiPosPolar, this->cfg_red_height_);
	}
	return roiPosPolar;
}

bool
LightCompassThread::isValidSuccessor(float px, float py, float cx, float cy){

	float dx,dy;
	dx = cx - px;
	dy = cy - py;

	if(std::sqrt( (dx * dx) + (dy * dy)) < this->cfg_lightDistanceAllowedBetweenFrames ){
		return true;
	}
	return false;

}


void
LightCompassThread::ResetInterface(){
	this->lightif->read();
	if( this->lightif->visibility_history() > -1 ){
		this->lightif->set_visibility_history(-1);
	} else {
		this->lightif->set_visibility_history(this->lightif->visibility_history()-1);
	}
	this->lightif->write();
}

void
LightCompassThread::UpdateInterface(double lightPosX, double lightPosY, fawkes::polar_coord_2d_t * lightPol){
	//wert in interface schreiben
	this->lightif->read();
	logger->log_debug(name(),"Position: %f %f Rotation: %f Radius: %f",lightPosX ,lightPosY ,lightPol->phi ,lightPol->r);
	double lightifValuse[3];
	lightifValuse[0] = this->lightif->translation(0);
	lightifValuse[1] = this->lightif->translation(1);
	lightifValuse[2] = this->lightif->translation(2);

	if(isValidSuccessor(lightifValuse[0], lightifValuse[1], lightPosX, lightPosY)){
		this->lightif->set_visibility_history(this->lightif->visibility_history()+1);
	}
	else{
		this->lightif->set_visibility_history(-1);
	}

	lightifValuse[0] = lightPosX;
	lightifValuse[1] = lightPosY;
	lightifValuse[2] = (double)lightPol->r;

	this->lightif->set_translation(lightifValuse);
	this->lightif->set_rotation(0,(double)lightPol->phi);
	this->lightif->write();

	//logger->log_info(name(), "X = %f, Y = %f, R = %f", lightifValuse[0], lightifValuse[1], lightifValuse[2]);
}

firevision::ROI* LightCompassThread::getBestROI(unsigned char* bufferYCbCr)
{
	//mögliche rois suchen
	scanline_->reset();
	this->classifier_white_->set_src_buffer(bufferYCbCr, this->img_width_, this->img_height_);
	std::list<firevision::ROI> *rois = this->classifier_white_->classify();

	logger->log_debug(name(),"Classifier ROIs count %i",rois->size());

	//rois die nicht "gut" sind löschen (zu groß / zu nah am Roboter)
	std::list<firevision::ROI> *roisGood = new std::list<firevision::ROI>();
	firevision::ROI *roi = NULL;
	while ( ! rois->empty() ) {

		roi = new firevision::ROI(rois->front());
		logger->log_debug(name(),"Rois: Position: %u, %u", roi->start.x ,  roi->start.y);
		drawROIInBuffer(this->buffer_filtered, this->img_width_, this->img_height_, roi);
		if(roi->get_height() * roi->get_width() > this->cfg_threashold_roiMaxSize_)
		{
			logger->log_debug(name(), "DELETED, roi size is %f", roi->get_height() * roi->get_width() );
		}
		else if( this->mirror_->getWorldPointRelative(roi->start.x,roi->start.y).r < this->cfg_robot_radius_)
		{
			logger->log_debug(name(), "DELETED: radius to robot is %f", this->mirror_->getWorldPointRelative(roi->start.x,roi->start.y).r);
		}
		else
		{
			roisGood->push_back(*roi);
		}
		rois->pop_front();
	}

	firevision::convert(cspace_from_, cspace_to_, cam_->buffer(), buffer_filtered, img_width_,img_height_);
	this->drawROIsInBuffer(this->buffer_filtered, this->img_width_, this->img_height_, roisGood);

	//setze ROI mit geringstem abstand zum roboter in return value
	if ( ! roisGood->empty() ) {
		roi = new firevision::ROI(roisGood->front());
		for (std::list<firevision::ROI>::iterator it = roisGood->begin(); it != roisGood->end(); ++it) {

			float newRoiDistance = this->mirror_->getWorldPointRelative((*it).start.x,(*it).start.y).r;
			float oldRoidDistance = this->mirror_->getWorldPointRelative(roi->start.x,roi->start.y).r;
			if (newRoiDistance < oldRoidDistance) {
				roi = new firevision::ROI(*it);
			}
		}
	} else {
		roi = NULL;
	}

	delete roisGood;
	delete rois;

	return roi;
}

void LightCompassThread::drawROIsInBuffer(unsigned char* buffer, unsigned int width, unsigned int height, std::list<firevision::ROI> *rois)
{
	this->filterROIDraw->set_src_buffer(buffer, firevision::ROI::full_image(width, height), 0);
	logger->log_debug(name(),"Drawing ROIs count %i",rois->size());
	for (std::list<firevision::ROI>::iterator it = rois->begin(); it != rois->end(); ++it) {
		drawROIInBuffer(buffer,width,height,&(*it));
	}
}

void LightCompassThread::drawROIInBuffer(unsigned char* buffer, unsigned int width, unsigned int height, firevision::ROI *roi)
{
	this->filterROIDraw->set_src_buffer(buffer, firevision::ROI::full_image(width, height), 0);

	this->filterROIDraw->set_dst_buffer(buffer, roi);
	this->filterROIDraw->set_style(firevision::FilterROIDraw::DASHED_HINT);
	this->filterROIDraw->apply();
}

fawkes::polar_coord_2d_t LightCompassThread::distanceCorrectionOfAmple(fawkes::polar_coord_2d_t polatCooredOfMessuredLight, float ampleHeight)
{
	float distance = polatCooredOfMessuredLight.r;

	fawkes::polar_coord_2d_t polatCooredOfAmplePosition(polatCooredOfMessuredLight);

	polatCooredOfAmplePosition.r = distance - ( (ampleHeight / this->cfg_mirror_height) * distance);

	return polatCooredOfAmplePosition;
}










