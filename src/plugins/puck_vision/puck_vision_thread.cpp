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

#include "puck_vision_thread.h"

PuckVisionThread::PuckVisionThread()
:	Thread("PuckVisionThread", Thread::OPMODE_WAITFORWAKEUP),
 	VisionAspect(VisionAspect::CYCLIC)
{
	this->cfg_prefix = "";
	this->cfg_camera = "";
	this->cfg_frame = "";

	this->cfg_cameraFactorHorizontal = 0;
	this->cfg_cameraFactorVertical = 0;

	this->cfg_cameraOffsetHorizontalRad = 0;
	this->cfg_cameraOffsetVertical = 0;

	this->img_width = 0;
	this->img_height = 0;

	this->cfg_color_y_min = 0;
	this->cfg_color_y_max = 0;
	this->cfg_color_u_min = 0;
	this->cfg_color_u_max = 0;
	this->cfg_color_v_min = 0;
	this->cfg_color_v_max = 0;

	this->bufferYCbCr = NULL;

	this->cspaceFrom = firevision::YUV422_PLANAR;
	this->cspaceTo = firevision::YUV422_PLANAR;

	this->camera = NULL;
	this->scanline = NULL;
	this->colorModel = NULL;
	this->classifierExpected = NULL;
	this->shmBufferYCbCr = NULL;

	this->nearestPuck = NULL;

	this->drawer = NULL;

	this->cfg_debugMessagesActivated = false;
}

void
PuckVisionThread::init()
{
	logger->log_info(name(), "puck_vision: starts up");

	this->cfg_prefix = "/plugins/puck_vision/";

	this->cfg_frame  = this->config->get_string((this->cfg_prefix + "frame").c_str());

	this->cfg_camera = this->config->get_string((this->cfg_prefix + "camera").c_str());
	this->cfg_cameraFactorHorizontal = this->config->get_float((this->cfg_prefix + "camera_factor_horizontal").c_str());
	this->cfg_cameraFactorVertical = this->config->get_float((this->cfg_prefix + "camera_factor_vertical").c_str());

	this->cfg_cameraOffsetHorizontalRad = this->config->get_float((this->cfg_prefix + "camera_offset_horizontal_rad").c_str());
	this->cfg_cameraOffsetVertical = this->config->get_int((this->cfg_prefix + "camera_offset_vertical").c_str());

	this->cfg_debugMessagesActivated = this->config->get_bool((this->cfg_prefix + "show_debug_messages").c_str());
	this->cfg_paintROIsActivated = this->config->get_bool((this->cfg_prefix + "draw_rois").c_str());

	this->cfg_puck_radius = this->config->get_float((this->cfg_prefix + "puck_radius").c_str());

	this->cfg_p1_distance =(double) this->config->get_float((this->cfg_prefix + "p1_distance").c_str());
	this->cfg_p1_width = (double)this->config->get_float((this->cfg_prefix + "p1_width").c_str());
	this->cfg_p2_distance = (double)this->config->get_float((this->cfg_prefix + "p2_distance").c_str());
	this->cfg_p2_width =(double) this->config->get_float((this->cfg_prefix + "p2_width").c_str());
	this->m = double( cfg_p2_distance - cfg_p1_distance )/double( cfg_p2_width - cfg_p1_width) ;
	logger->log_info(name(),"Points P1(%f,%f) P2(%f,%f)  =>  m = %f",cfg_p1_width, cfg_p1_distance, cfg_p2_width, cfg_p2_distance,m);




	this->cfg_color_y_min = this->config->get_uint((this->cfg_prefix + "color_y_min").c_str());
	this->cfg_color_y_max = this->config->get_uint((this->cfg_prefix + "color_y_max").c_str());
	this->cfg_color_u_min = this->config->get_uint((this->cfg_prefix + "color_u_min").c_str());
	this->cfg_color_u_max = this->config->get_uint((this->cfg_prefix + "color_u_max").c_str());
	this->cfg_color_v_min = this->config->get_uint((this->cfg_prefix + "color_v_min").c_str());
	this->cfg_color_v_max = this->config->get_uint((this->cfg_prefix + "color_v_max").c_str());

	std::string shmID = this->config->get_string((this->cfg_prefix + "shm_image_id").c_str());

	this->camera = vision_master->register_for_camera(this->cfg_camera.c_str(), this);

	this->img_width = this->camera->pixel_width();
	this->img_height = this->camera->pixel_height();

	this->cspaceFrom = this->camera->colorspace();

	this->scanline = new firevision::ScanlineGrid( this->img_width, this->img_height, 1, 1 );

	this->colorModel = new firevision::ColorModelRange( cfg_color_y_min, cfg_color_y_max,
														cfg_color_u_min, cfg_color_u_max,
														cfg_color_v_min, cfg_color_v_max);

	this->classifierExpected = new firevision::SimpleColorClassifier(
			this->scanline,														//scanmodel
			this->colorModel,													//colorModel
			30,																	//num_min_points
			0,																	//box_extend
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
	this->nearestPuck = blackboard->open_for_writing<fawkes::Position3DInterface>(
	this->config->get_string((this->cfg_prefix + "nearest_puck_if").c_str()).c_str());
	this->nearestPuck->set_frame(this->cfg_frame.c_str());
	this->nearestPuck->write();

	//ROIs
	this->drawer = new firevision::FilterROIDraw();

	roi_center.start.x=(img_width/2) -2;
	roi_center.start.y=(img_height/2) -2;
	roi_center.width=4;
	roi_center.height=4;
	roi_center.image_height=img_height;
	roi_center.image_width=img_width;

	logger->log_debug(name(), "end of init()");
}

void
PuckVisionThread::finalize()													//TODO check if everthing gets deleted
{
	logger->log_debug(name(), "finalize starts");

	vision_master->unregister_thread(this);

	delete this->camera;
	delete this->scanline;
	delete this->colorModel;
	delete this->classifierExpected;
	delete this->shmBufferYCbCr;

	blackboard->close(this->nearestPuck);

	logger->log_info(name(), "finalize ends");
}

void
PuckVisionThread::loop()
{
	camera->capture();
		firevision::convert(this->cspaceFrom,
				this->cspaceTo,
				this->camera->buffer(),
				this->bufferYCbCr,
				this->img_width,
				this->img_height);
	this->camera->dispose_buffer();

	scanline->reset();
	classifierExpected->set_src_buffer(bufferYCbCr, this->img_width, this->img_height);
	std::list<firevision::ROI> *rois = this->classifierExpected->classify();
	//logger->log_info(name(),"Items found %i",rois->size());
	firevision::ROI* puck = getBiggestRoi(rois);
	updateInterface(puck);

	if(cfg_paintROIsActivated){
		while(!rois->empty()){
			drawROIIntoBuffer(rois->front(), firevision::FilterROIDraw::DASHED_HINT);
			rois->pop_front();
		}
		drawROIIntoBuffer(roi_center, firevision::FilterROIDraw::INVERTED);
	}



	delete rois;
}

firevision::ROI* PuckVisionThread::getBiggestRoi( std::list<firevision::ROI>* roiList) {
	firevision::ROI* biggestRoi=NULL;

	for (std::list<firevision::ROI>::iterator it = roiList->begin();
			it != roiList->end(); ++it) {
		if (biggestRoi == NULL || biggestRoi->width * biggestRoi->height
				< (*it).width * (*it).height) {
			biggestRoi = &(*it);
		}
	}
	return biggestRoi;
}




PuckVisionThread::~PuckVisionThread()
{
	// TODO Auto-generated destructor stub
}

fawkes::polar_coord_2d_t
PuckVisionThread::transformCoordinateSystem(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to)
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

void PuckVisionThread::cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y) {
	pol.phi = atan2f(y, x);
	pol.r = sqrtf(x * x + y * y);

	if (this->cfg_debugMessagesActivated) {
		logger->log_debug(name(), "x: %f; y: %f", x, y);
		logger->log_debug(name(), "Calculated r: %f; phi: %f", pol.r, pol.phi);
	}
}

void PuckVisionThread::polToCart(float &x, float &y,fawkes::polar_coord_2d_t pol){
	x = pol.r * std::cos(pol.phi);
	y = pol.r * std::sin(pol.phi);
}

void
PuckVisionThread::drawROIIntoBuffer(firevision::ROI roi, firevision::FilterROIDraw::border_style_t borderStyle)
{
	this->drawer->set_src_buffer(this->bufferYCbCr, firevision::ROI::full_image(this->img_width, this->img_height), 0);
	this->drawer->set_dst_buffer(this->bufferYCbCr, &roi);
	this->drawer->set_style(borderStyle);
	this->drawer->apply();

	if (this->cfg_debugMessagesActivated) {
		logger->log_debug(name(), "drawed element in buffer");
	}
}


std::list<firevision::ROI>*
PuckVisionThread::classifyInRoi(firevision::ROI searchArea, firevision::Classifier *classifier)
{
	this->scanline->reset();
	this->scanline->set_roi(&searchArea);

	classifier->set_src_buffer(this->bufferYCbCr, this->img_width, this->img_height);

	std::list<firevision::ROI> *ROIs = classifier->classify();

	return ROIs;
}

fawkes::polar_coord_2d_t
PuckVisionThread::positionFromRoi(firevision::ROI* roi){

	// x = width, y = distance
	fawkes::polar_coord_2d_t puck_position;
	puck_position.r = distanceCorrection(roi->width) + cfg_puck_radius;
	//puck_position.r = m * (roi->width - cfg_p1_width) +  cfg_p1_distance + cfg_puck_radius;
	logger->log_info(name(),"Distance = %f, rois: center (%u, %u) width,height (%u, %u)",puck_position.r , roi->start.x + (roi->width/2),roi->start.y + (roi->height/2), roi->width, roi->height);

	float pixelPerRadHorizonal = this->img_width / this->cfg_cameraFactorHorizontal;

	puck_position.phi = ((roi->start.x + roi->width/2)	- this->img_width / 2)
					/ pixelPerRadHorizonal;

	return puck_position;
}

double
PuckVisionThread::distanceCorrection(double x_in)
{
	//front
//	double temp;
//	temp = 0.0;
//	// coefficients
//	double a = 8.4095039634708191E-01;
//	double b = -7.9038576694095385E-03;
//	temp = a * exp(b*x_in);
//	return temp;
//	//top
	 double temp;
	temp = 0.0;
	// coefficients
	double a = 1.6793033074297550E+00;
	double b = -1.6433635226927507E-02;
	temp = a * exp(b*x_in);
	return temp;
}

void
PuckVisionThread::updateInterface(firevision::ROI* puck){
	nearestPuck->read();
	if(puck==NULL){
		nearestPuck->set_visibility_history(-1);
	}
	else{
		fawkes::polar_coord_2d_t polar = positionFromRoi(puck);

		float x,y;

		polToCart(x,y,polar);

		nearestPuck->set_visibility_history(nearestPuck->visibility_history()+1);
		nearestPuck->set_translation(0,x);
		nearestPuck->set_translation(1,y);
		nearestPuck->set_rotation(0,polar.phi);
		nearestPuck->set_rotation(1,polar.r);
	}
	nearestPuck->write();
}











