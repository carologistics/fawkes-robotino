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
	cfg_prefix_ = "";
	cfg_camera_ = "";
	cfg_frame_ = "";

	img_width_ = 0;
	img_height_ = 0;

	cfg_distance_function_a_ = 0;
	cfg_distance_function_b_ = 0;

	cfg_camera_opening_angle_horizontal_ = 0;
	cfg_camera_opening_angle_vertical_ = 0;

	buffer_ = NULL;
	shm_buffer_ = NULL;

	cspaceFrom_ = firevision::YUV422_PLANAR;
	cspaceTo_ = firevision::YUV422_PLANAR;

	cam_ = NULL;
	scanline_ = NULL;

	cm_yellow_ = NULL;
	cm_red_ = NULL;
	cm_blue_ = NULL;
	cm_green_ = NULL;

	classifier_yellow_ = NULL;
	classifier_red_ = NULL;
	classifier_green_ = NULL;
	classifier_blue_ = NULL;
	classifier_similarity_ = NULL;

	cls_red_.colormodel = NULL;

	drawer_ = NULL;

	cfg_debugMessagesActivated_ = false;
}

void
PuckVisionThread::init()
{
	logger->log_info(name(), "starts init");

	cfg_prefix_= "/plugins/puck_vision/";
	cfg_prefix_static_transforms_ = "/plugins/static-transforms/transforms/cam_puck/";

	cfg_frame_  = config->get_string((cfg_prefix_ + "frame").c_str());
	cfg_camera_ = config->get_string((cfg_prefix_ + "camera").c_str());

	cfg_debugMessagesActivated_ = config->get_bool((cfg_prefix_ + "show_debug_messages").c_str());
	cfg_paintROIsActivated_ = config->get_bool((cfg_prefix_ + "draw_rois").c_str());

	cfg_puck_radius_ = config->get_float((cfg_prefix_ + "puck_radius").c_str());

	//Config Value for the classifier mode
	cfg_colormodel_mode_ = config->get_string((cfg_prefix_ + "colormodel_mode").c_str());
	// Configure Similiraty classifier
	cls_red_.cfg_ref_col = config->get_uints((cfg_prefix_ + "/reference_color").c_str());
	cls_red_.cfg_chroma_thresh = config->get_int((cfg_prefix_ + "/chroma_thresh").c_str());
	cls_red_.cfg_sat_thresh = config->get_int((cfg_prefix_ + "/saturation_thresh").c_str());
	cls_red_.cfg_roi_min_points = config->get_int((cfg_prefix_ + "/min_points").c_str());
	cls_red_.cfg_roi_basic_size = config->get_int((cfg_prefix_ + "/basic_roi_size").c_str());
	cls_red_.cfg_roi_neighborhood_min_match = config->get_int((cfg_prefix_ + "/neighborhood_min_match").c_str());
	cls_red_.cfg_scangrid_x_offset = config->get_int((cfg_prefix_ + "/scangrid_x_offset").c_str());
	cls_red_.cfg_scangrid_y_offset = config->get_int((cfg_prefix_ + "/scangrid_y_offset").c_str());

	cls_red_.color_class = new firevision::ColorModelSimilarity::color_class_t(cls_red_.color_expect, cls_red_.cfg_ref_col, cls_red_.cfg_chroma_thresh, cls_red_.cfg_sat_thresh);

	cls_red_.color_class->chroma_threshold = cls_red_.cfg_chroma_thresh;
	cls_red_.color_class->saturation_threshold = cls_red_.cfg_sat_thresh;
	cls_red_.color_class->set_reference(cls_red_.cfg_ref_col);

	cls_red_.colormodel = new firevision::ColorModelSimilarity();
	cls_red_.colormodel->add_color(cls_red_.color_class);

	cfg_colormap_file_yellow_ = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + "colormap_file_yellow").c_str());
	cfg_colormap_file_red_ = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + "colormap_file_red").c_str());
	cfg_colormap_file_green_ = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + "colormap_file_green").c_str());
	cfg_colormap_file_blue_ = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + "colormap_file_blue").c_str());

	cfg_camera_position_x_ = config->get_float(( cfg_prefix_static_transforms_ + "trans_x").c_str());
	cfg_camera_position_y_ =  config->get_float(( cfg_prefix_static_transforms_ + "trans_y").c_str());
	cfg_camera_position_z_ =  config->get_float(( cfg_prefix_static_transforms_ + "trans_z").c_str());
	cfg_camera_position_pitch_ =  config->get_float(( cfg_prefix_static_transforms_ + "rot_pitch").c_str());


	logger->log_debug(name(), "cam transform X: %f y: %f z: %f pitch: %f",cfg_camera_position_x_, cfg_camera_position_y_, cfg_camera_position_z_, cfg_camera_position_pitch_);

	cfg_camera_opening_angle_horizontal_ = config->get_float((cfg_prefix_ + "camera_opening_angle_horizontal").c_str());
	cfg_camera_opening_angle_vertical_ = config->get_float((cfg_prefix_ + "camera_opening_angle_vertical").c_str());

	logger->log_debug(name(),"Camera opening angles Horizontal: %f Vertical: %f", cfg_camera_opening_angle_horizontal_, cfg_camera_opening_angle_vertical_);

	cm_yellow_ = new firevision::ColorModelLookupTable(cfg_colormap_file_yellow_.c_str(),"puckvision-yellow-colormap", true /* destroy on delete */);
	cm_red_ = new firevision::ColorModelLookupTable(cfg_colormap_file_red_.c_str(),"puckvision-red-colormap", true /* destroy on delete */);
	cm_green_ = new firevision::ColorModelLookupTable(cfg_colormap_file_green_.c_str(),"puckvision-green-colormap", true /* destroy on delete */);
	cm_blue_ = new firevision::ColorModelLookupTable(cfg_colormap_file_blue_.c_str(),"puckvision-blue-colormap", true /* destroy on delete */);

	cfg_nr_puck_interfaces_ = config->get_uint((cfg_prefix_ + "nr_puck_interfaces").c_str());

	std::string shmID = config->get_string((cfg_prefix_ + "shm_image_id").c_str());

	cam_ = vision_master->register_for_camera(cfg_camera_.c_str(), this);

	img_width_ = cam_->pixel_width();
	img_height_ = cam_->pixel_height();

	cspaceFrom_ = cam_->colorspace();

	scanline_ = new firevision::ScanlineGrid( img_width_, img_height_, 2, 2 );


	classifier_yellow_ = new firevision::SimpleColorClassifier(
			scanline_,															//scanmodel
			cm_yellow_,															//colorModel
			30,																	//num_min_points
			0,																	//box_extend
			false,																//upward
			2,																	//neighberhoud_min_match
			0																	//grow_by
			);

	classifier_red_ = new firevision::SimpleColorClassifier(
			scanline_,														//scanmodel
			cm_red_,														//colorModel
			30,																//num_min_points
			0,																//box_extend
			false,															//upward
			2,																//neighberhoud_min_match
			0																//grow_by
			);

	classifier_green_ = new firevision::SimpleColorClassifier(
			scanline_,														//scanmodel
			cm_green_,														//colorModel
			30,																//num_min_points
			0,																//box_extend
			false,															//upward
			2,																//neighberhoud_min_match
			0																//grow_by
			);

	classifier_blue_ = new firevision::SimpleColorClassifier(
			scanline_,														//scanmodel
			cm_blue_,														//colorModel
			30,																//num_min_points
			0,																//box_extend
			false,															//upward
			2,																//neighberhoud_min_match
			0																//grow_by
			);

	classifier_similarity_ = new firevision::SimpleColorClassifier(
			scanline_,														//scanmodel
			cls_red_.colormodel,														//colorModel
			30,																//num_min_points
			0,																//box_extend
			false,															//upward
			2,																//neighberhoud_min_match
			0																//grow_by
	);


	// SHM image buffer
	shm_buffer_ = new firevision::SharedMemoryImageBuffer(
			shmID.c_str(),
			cspaceTo_,
			img_width_,
			img_height_
			);
	if (!shm_buffer_->is_valid()) {
		throw fawkes::Exception("Shared memory segment not valid");
	}
	shm_buffer_->set_frame_id(cfg_frame_.c_str());

	buffer_ = shm_buffer_->buffer();


	//open interfaces
	puckInterface_ = blackboard->open_for_writing<fawkes::PuckVisionInterface>(
			config->get_string((cfg_prefix_ + "puck_if").c_str()).c_str()
			);

	puckInterface_->set_frame(cfg_frame_.c_str());

	// Calculate visible Area
	angle_horizontal_to_opening_ = (1.57 - cfg_camera_position_pitch_) - (cfg_camera_opening_angle_vertical_ / 2) ;
	visible_lenght_x_in_m_ = cfg_camera_position_z_ * (tan( cfg_camera_opening_angle_vertical_ + angle_horizontal_to_opening_) - tan(angle_horizontal_to_opening_));

	offset_cam_x_to_groundplane_ = cfg_camera_position_z_ * tan(angle_horizontal_to_opening_);

	visible_lenght_y_in_m_ = cfg_camera_position_z_ * tan (cfg_camera_opening_angle_horizontal_ /2 );

	logger->log_debug(name(), "Visible X: %f y: %f Offset cam groundplane: %f angle (horizontal/opening): %f ",visible_lenght_x_in_m_, visible_lenght_y_in_m_ , offset_cam_x_to_groundplane_, angle_horizontal_to_opening_);


	//ROIs
	drawer_ = new firevision::FilterROIDraw();

	//Defines the search area
	roi_center_.start.x=(img_width_/2) -2;
	roi_center_.start.y=(img_height_/2) -2;
	roi_center_.width=4;
	roi_center_.height=4;
	roi_center_.image_height=img_height_;
	roi_center_.image_width=img_width_;

	logger->log_debug(name(), "end of init()");
}


void
PuckVisionThread::drawRois(std::list<firevision::ROI>* rois_) {
	for(std::list<firevision::ROI>::iterator list_iter = rois_->begin();
	    list_iter != rois_->end(); list_iter++)
	{
		drawROIIntoBuffer(*list_iter, firevision::FilterROIDraw::DASHED_HINT);
	}
}

void
PuckVisionThread::mergeWithColorInformation(firevision::color_t color,
		std::list<firevision::ROI>* rois_color_,
		std::list<firevision::ROI>* rois_all) {
	while(rois_color_->size() !=0 ) {
		rois_color_->front().color = color;
		rois_all->push_front(rois_color_->front());
		rois_color_->pop_front();
	}
}

fawkes::PuckVisionInterface::PuckColor
PuckVisionThread::getPuckInterfaceColor(firevision::ROI* roi){
	switch (roi->color) {
		case firevision::C_BLUE:
			return fawkes::PuckVisionInterface::C_BLUE;
		case firevision::C_RED:
			return fawkes::PuckVisionInterface::C_RED;
		case firevision::C_GREEN:
			return fawkes::PuckVisionInterface::C_GREEN;
		case firevision::C_YELLOW:
			return fawkes::PuckVisionInterface::C_YELLOW;
		case firevision::C_BLACK:
			return fawkes::PuckVisionInterface::C_BLACK;
		case firevision::C_WHITE:
			return fawkes::PuckVisionInterface::C_WHITE;
		default:
			return fawkes::PuckVisionInterface::C_UNKNOWN;
	}
}

void
PuckVisionThread::loop()
{
	cam_->capture();
	firevision::convert(cspaceFrom_,
				cspaceTo_,
				cam_->buffer(),
				buffer_,
				img_width_,
				img_height_);
	cam_->dispose_buffer();

	std::list<firevision::ROI> *rois_all_ = new std::list<firevision::ROI>();

	if (cfg_colormodel_mode_ == "colormap"){
		scanline_->reset();
		classifier_yellow_->set_src_buffer(buffer_, img_width_, img_height_);
		std::list<firevision::ROI> *rois_yellow_ = classifier_yellow_->classify();

		scanline_->reset();
		classifier_red_->set_src_buffer(buffer_, img_width_, img_height_);
		std::list<firevision::ROI> *rois_red_ = classifier_red_->classify();

		scanline_->reset();
		classifier_green_->set_src_buffer(buffer_, img_width_, img_height_);
		std::list<firevision::ROI> *rois_green_ = classifier_green_->classify();

		scanline_->reset();
		classifier_blue_->set_src_buffer(buffer_, img_width_, img_height_);
		std::list<firevision::ROI> *rois_blue_ = classifier_blue_->classify();

		mergeWithColorInformation(firevision::C_BLUE, rois_blue_, rois_all_);
		mergeWithColorInformation(firevision::C_GREEN, rois_green_, rois_all_);
		mergeWithColorInformation(firevision::C_YELLOW, rois_yellow_, rois_all_);
		mergeWithColorInformation(firevision::C_RED, rois_red_, rois_all_);

		delete rois_green_;
		delete rois_yellow_;
		delete rois_red_;
		delete rois_blue_;

	} else if(cfg_colormodel_mode_ == "similarity"){
		scanline_->reset();
		classifier_similarity_->set_src_buffer(buffer_, img_width_, img_height_);
		std::list<firevision::ROI> *rois_similarity_ = classifier_similarity_->classify();

		mergeWithColorInformation(firevision::C_MAGENTA, rois_similarity_, rois_all_);

		delete rois_similarity_;
	}


	if(cfg_paintROIsActivated_){
			drawRois(rois_all_);
	}

	updateInterface(rois_all_);

	delete rois_all_;
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
	cartToPol(polErrorReturnValue, cartFrom.x, cartFrom.y);

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
		cartToPol(polTo, toX, toY);

		if (cfg_debugMessagesActivated_) {
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

	if (cfg_debugMessagesActivated_) {
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
	drawer_->set_src_buffer(buffer_, firevision::ROI::full_image(img_width_, img_height_), 0);
	drawer_->set_dst_buffer(buffer_, &roi);
	drawer_->set_style(borderStyle);
	drawer_->apply();

	if (cfg_debugMessagesActivated_) {
		logger->log_debug(name(), "drawed element in buffer");
	}
}


std::list<firevision::ROI>*
PuckVisionThread::classifyInRoi(firevision::ROI searchArea, firevision::Classifier *classifier)
{
	scanline_->reset();
	scanline_->set_roi(&searchArea);

	classifier->set_src_buffer(buffer_, img_width_, img_height_);

	std::list<firevision::ROI> *ROIs = classifier->classify();

	return ROIs;
}

float
PuckVisionThread::getX(firevision::ROI* roi){
	// Distance X
	int roiPositionY = img_height_ - ( roi->start.y +roi->height/2 );
	float angle = ( (float)roiPositionY * cfg_camera_opening_angle_vertical_ ) / (float)img_height_;
	float distance_x = cfg_camera_position_z_ * (tan( angle + angle_horizontal_to_opening_) - tan(angle_horizontal_to_opening_));

	//logger->log_debug(name(),"angle: %f, image_size x: %i y: %i,roi x: %i y: %i", (angle*180) / M_PI, img_width_, img_height_, roi->start.x, roi->start.y);

	return distance_x + offset_cam_x_to_groundplane_ + cfg_camera_position_x_;
}

float
PuckVisionThread::getY(firevision::ROI* roi){
	// Left-Right
	int roiPositionX = ( roi->start.x + roi->width/2 );
	float alpha = ((((float)img_width_/2) - (float)roiPositionX) / ((float)img_width_ /2 )) * (cfg_camera_opening_angle_horizontal_/2);
	float x = getX(roi);
	float position_y_in_m = sin(alpha * x);

	//logger->log_debug(name(),"angle: %f, roi pos X: %i, distance y: %f, cfg %f", alpha, roiPositionX, position_y_in_m, cfg_camera_opening_angle_horizontal_);

	return position_y_in_m;
}

int
PuckVisionThread::getVisibilityHistory(fawkes::polar_coord_2d_t polar,
		fawkes::PuckVisionInterface::PuckColor colorRoi,
		fawkes::PuckVisionInterface::PuckColor colorInterface,
		float interface_phi, float interface_r,
		int interface_visibility)
{
	bool toBigRotation = polar.phi - interface_phi > 0.1;
	bool toBigMovement = polar.r - interface_r > 0.1;
	bool colorChanged = colorRoi != colorInterface;
	if (toBigRotation || toBigMovement || colorChanged
			|| colorRoi == fawkes::PuckVisionInterface::C_UNKNOWN) {
		return -1;
	} else {
		return ++interface_visibility;
	}
}

void
PuckVisionThread::updateInterface(std::list<firevision::ROI>* rois){
	puckInterface_->read();
	puckInterface_->set_frame(cfg_frame_.c_str());

	if (cfg_debugMessagesActivated_) {
		logger->log_info(name(),"Roi-list size %i",rois->size());
	}
	for(unsigned int i=1; i<4; ++i){

		fawkes::polar_coord_2d_t polar;
		polar.phi = 0;
		polar.r = 0;
		float x = 0;
		float y = 0;
		fawkes::PuckVisionInterface::PuckColor color = fawkes::PuckVisionInterface::C_UNKNOWN;

		firevision::ROI* roi = getBiggestRoi(rois);

		if(roi != NULL){
			x = getX(roi);
			y = getY(roi);
			cartToPol(polar,x,y);
			if (cfg_debugMessagesActivated_) {
				logger->log_info(name(),"%i x: %f y: %f r: %f phi: %f",i, x,  y, polar.r, polar.phi);
			}
			color = getPuckInterfaceColor(roi);
			rois->remove(roi);
		}
		switch (i) {
			case 1:
				puckInterface_->set_puck1_visibility_history(
					getVisibilityHistory(
						polar,
						color,
						puckInterface_->puck1_color(),
						puckInterface_->puck1_polar(0),
						puckInterface_->puck1_polar(1),
						puckInterface_->puck1_visibility_history()
					)
				);
				puckInterface_->set_puck1_color(color);
				puckInterface_->set_puck1_translation(0,x);
				puckInterface_->set_puck1_translation(1,y);
				puckInterface_->set_puck1_polar(0,polar.phi);
				puckInterface_->set_puck1_polar(1,polar.r);
				break;

			case 2:
				puckInterface_->set_puck2_visibility_history(
					getVisibilityHistory(
						polar,
						color,
						puckInterface_->puck2_color(),
						puckInterface_->puck2_polar(0),
						puckInterface_->puck2_polar(1),
						puckInterface_->puck2_visibility_history()
					)
				);
				puckInterface_->set_puck2_color(color);
				puckInterface_->set_puck2_translation(0,x);
				puckInterface_->set_puck2_translation(1,y);
				puckInterface_->set_puck2_polar(0,polar.phi);
				puckInterface_->set_puck2_polar(1,polar.r);
				break;

			case 3:
				puckInterface_->set_puck3_visibility_history(
					getVisibilityHistory(
						polar,
						color,
						puckInterface_->puck3_color(),
						puckInterface_->puck3_polar(0),
						puckInterface_->puck3_polar(1),
						puckInterface_->puck3_visibility_history()
					)
				);
				puckInterface_->set_puck3_color(color);
				puckInterface_->set_puck3_translation(0,x);
				puckInterface_->set_puck3_translation(1,y);
				puckInterface_->set_puck3_polar(0,polar.phi);
				puckInterface_->set_puck3_polar(1,polar.r);
				break;
		}

	}
	puckInterface_->write();
}

void
PuckVisionThread::finalize()													//TODO check if everthing gets deleted
{
	logger->log_debug(name(), "finalize starts");

	vision_master->unregister_thread(this);
	blackboard->close(puckInterface_);

	delete cam_;
	delete scanline_;
	delete cm_blue_;
	delete cm_yellow_;
	delete cm_red_;
	delete cm_green_;

	delete classifier_blue_;
	delete classifier_red_;
	delete classifier_green_;
	delete classifier_yellow_;
	delete classifier_similarity_;
	delete shm_buffer_;

	delete cls_red_.colormodel;


	logger->log_info(name(), "finalize ends");
}








