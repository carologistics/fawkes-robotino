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

#define CFG_PREFIX "/plugins/puck_vision/"
#define CFG_TRANSFORM_PREFIX "/plugins/static-transforms/transforms/cam_puck/"

PuckVisionThread::PuckVisionThread()
:	Thread("PuckVisionThread", Thread::OPMODE_WAITFORWAKEUP),
 	VisionAspect(VisionAspect::CYCLIC),
    ConfigurationChangeHandler(CFG_PREFIX)
//,   ConfigurationChangeHandler(CFG_TRANSFORM_PREFIX)
{
	cfg_prefix_ = CFG_PREFIX;
	cfg_frame_ = "";

	camera_info_.cfg_camera_ = "";
	camera_info_.img_width_ = 0;
	camera_info_.img_height_ = 0;
	camera_info_.opening_angle_horizontal_ = 0;
	camera_info_.opening_angle_vertical_ = 0;

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
	cls_red_.color_class = NULL;

	switchInterface_ = NULL;
	no_pucK_ = NULL;

	drawer_ = NULL;

	cfg_paintROIsActivated_ = false;
	cfg_debugMessagesActivated_ = false;
	cfg_changed_ = false;
}
/*
 * Create a new puck interface for writing and append it to puck_interfaces_ vector
 */
void
PuckVisionThread::createPuckInterface(){
	fawkes::Position3DInterface* puck_if_ = NULL;

	std::string interface_name = cfg_prefix_ + "puck_"+std::to_string(puck_interfaces_.size());
	try {
		puck_if_ = blackboard->open_for_writing<fawkes::Position3DInterface>(interface_name.c_str() + puck_interfaces_.size());
		puck_if_->set_visibility_history(-9999);
		puck_if_->set_frame(cfg_frame_.c_str());
		puck_if_->write();
		puck_interfaces_.push_back(puck_if_);
	}catch (std::exception &e) {
		finalize();
		throw;
	  }
}

void PuckVisionThread::loadConfig(){
	logger->log_info(name(), "loading config");
	cfg_prefix_ = CFG_PREFIX;
	cfg_prefix_static_transforms_ = CFG_TRANSFORM_PREFIX;

	cfg_frame_ = config->get_string((cfg_prefix_ + "frame").c_str());

	camera_info_.cfg_camera_ = config->get_string((cfg_prefix_ + "camera").c_str());
	camera_info_.position_x_ = config->get_float(( cfg_prefix_static_transforms_ + "trans_x").c_str());
	camera_info_.position_y_ =  config->get_float(( cfg_prefix_static_transforms_ + "trans_y").c_str());
	camera_info_.position_z_ =  config->get_float(( cfg_prefix_static_transforms_ + "trans_z").c_str());
	camera_info_.position_pitch_ =  config->get_float(( cfg_prefix_static_transforms_ + "rot_pitch").c_str());

	cfg_debugMessagesActivated_ = config->get_bool((cfg_prefix_ + "show_debug_messages").c_str());
	cfg_paintROIsActivated_ = config->get_bool((cfg_prefix_ + "draw_rois").c_str());

	puck_info_.radius_ = config->get_float((cfg_prefix_ + "puck_radius").c_str());

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

	if(cls_red_.color_class != NULL){
		delete cls_red_.color_class;
	}
	cls_red_.color_class = new firevision::ColorModelSimilarity::color_class_t(cls_red_.color_expect, cls_red_.cfg_ref_col, cls_red_.cfg_chroma_thresh, cls_red_.cfg_sat_thresh);


	cls_red_.color_class->chroma_threshold = cls_red_.cfg_chroma_thresh;
	cls_red_.color_class->saturation_threshold = cls_red_.cfg_sat_thresh;
	cls_red_.color_class->set_reference(cls_red_.cfg_ref_col);

	if(cls_red_.colormodel != NULL){
		delete cls_red_.colormodel;
	}
	cls_red_.colormodel = new firevision::ColorModelSimilarity();
	cls_red_.colormodel->add_color(cls_red_.color_class);

	cfg_colormap_file_yellow_ = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + "colormap_file_yellow").c_str());
	cfg_colormap_file_red_ = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + "colormap_file_red").c_str());
	cfg_colormap_file_green_ = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + "colormap_file_green").c_str());
	cfg_colormap_file_blue_ = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + "colormap_file_blue").c_str());

	camera_info_.opening_angle_horizontal_ = config->get_float((cfg_prefix_ + "camera_opening_angle_horizontal").c_str());
	camera_info_.opening_angle_vertical_ = config->get_float((cfg_prefix_ + "camera_opening_angle_vertical").c_str());

	logger->log_debug(name(),"Camera opening angles Horizontal: %f Vertical: %f", camera_info_.opening_angle_horizontal_, camera_info_.opening_angle_vertical_);

	if(cm_yellow_ != NULL){
			delete cm_yellow_;
	}
	cm_yellow_ = new firevision::ColorModelLookupTable(cfg_colormap_file_yellow_.c_str(),"puckvision-yellow-colormap", true /* destroy on delete */);

	if(cm_red_ != NULL){
			delete cm_red_;
	}
	cm_red_ = new firevision::ColorModelLookupTable(cfg_colormap_file_red_.c_str(),"puckvision-red-colormap", true /* destroy on delete */);

	if(cm_green_ != NULL){
			delete cm_green_;
	}
	cm_green_ = new firevision::ColorModelLookupTable(cfg_colormap_file_green_.c_str(),"puckvision-green-colormap", true /* destroy on delete */);

	if(cm_blue_ != NULL){
			delete cm_blue_;
	}
	cm_blue_ = new firevision::ColorModelLookupTable(cfg_colormap_file_blue_.c_str(),"puckvision-blue-colormap", true /* destroy on delete */);


	if(scanline_ != NULL){
		delete scanline_;
	}
	scanline_ = new firevision::ScanlineGrid( camera_info_.img_width_, camera_info_.img_height_, 2, 2 );

	if(classifier_yellow_ != NULL){
		delete classifier_yellow_;
	}
	classifier_yellow_ = new firevision::SimpleColorClassifier(
			scanline_,															//scanmodel
			cm_yellow_,															//colorModel
			30,																	//num_min_points
			0,																	//box_extend
			false,																//upward
			2,																	//neighberhoud_min_match
			0																	//grow_by
			);

	if(classifier_red_ != NULL){
		delete classifier_red_;
	}
	classifier_red_ = new firevision::SimpleColorClassifier(
			scanline_,														//scanmodel
			cm_red_,														//colorModel
			30,																//num_min_points
			0,																//box_extend
			false,															//upward
			2,																//neighberhoud_min_match
			0																//grow_by
			);

	if(classifier_green_ != NULL){
		delete classifier_green_;
	}
	classifier_green_ = new firevision::SimpleColorClassifier(
			scanline_,														//scanmodel
			cm_green_,														//colorModel
			30,																//num_min_points
			0,																//box_extend
			false,															//upward
			2,																//neighberhoud_min_match
			0																//grow_by
			);

	if(classifier_blue_ != NULL){
		delete classifier_blue_;
	}
	classifier_blue_ = new firevision::SimpleColorClassifier(
			scanline_,														//scanmodel
			cm_blue_,														//colorModel
			30,																//num_min_points
			0,																//box_extend
			false,															//upward
			2,																//neighberhoud_min_match
			0																//grow_by
			);

	if(classifier_similarity_ != NULL){
		delete classifier_similarity_;
	}
	classifier_similarity_ = new firevision::SimpleColorClassifier(
			scanline_,														//scanmodel
			cls_red_.colormodel,														//colorModel
			30,																//num_min_points
			0,																//box_extend
			false,															//upward
			2,																//neighberhoud_min_match
			0																//grow_by
	);


	if(cam_ != NULL){
		cam_->stop();
		cam_->flush();
		cam_->dispose_buffer();
		cam_->close();
		delete cam_;
		cam_ = NULL;
	}
	cam_ = vision_master->register_for_camera(camera_info_.cfg_camera_.c_str(), this);
	cam_->start();
	cam_->open();
	camera_info_.img_width_ = cam_->pixel_width();
	camera_info_.img_height_ = cam_->pixel_height();

	cspaceFrom_ = cam_->colorspace();

	// SHM image buffer
	if(shm_buffer_ != NULL){
		delete shm_buffer_;
		shm_buffer_ = NULL;
		buffer_ = NULL;
	}
	std::string shmID = config->get_string((cfg_prefix_ + "shm_image_id").c_str());
	shm_buffer_ = new firevision::SharedMemoryImageBuffer(
			shmID.c_str(),
			cspaceTo_,
			camera_info_.img_width_,
			camera_info_.img_height_
			);

	if (!shm_buffer_->is_valid()) {
		delete shm_buffer_;
		delete cam_;
		shm_buffer_ = NULL;
		cam_ = NULL;
		throw fawkes::Exception("Shared memory segment not valid");
	}
	shm_buffer_->set_frame_id(cfg_frame_.c_str());

	buffer_ = shm_buffer_->buffer();

	// Calculate visible Area
	camera_info_.angle_horizontal_to_opening_ = (1.57 - camera_info_.position_pitch_) - (camera_info_.opening_angle_vertical_ / 2) ;
	camera_info_.visible_lenght_x_in_m_ = camera_info_.position_z_ * (tan( camera_info_.opening_angle_vertical_ + camera_info_.angle_horizontal_to_opening_) - tan(camera_info_.angle_horizontal_to_opening_));

	camera_info_.offset_cam_x_to_groundplane_ = camera_info_.position_z_ * tan(camera_info_.angle_horizontal_to_opening_);

	camera_info_.visible_lenght_y_in_m_ = camera_info_.position_z_ * tan (camera_info_.opening_angle_horizontal_ /2 );

	//Defines the search area
	roi_center_.start.x=(camera_info_.img_width_/2) -2;
	roi_center_.start.y=(camera_info_.img_height_/2) -2;
	roi_center_.width=4;
	roi_center_.height=4;
	roi_center_.image_height=camera_info_.img_height_;
	roi_center_.image_width=camera_info_.img_width_;

	logger->log_debug(name(), "Visible X: %f y: %f Offset cam groundplane: %f angle (horizontal/opening): %f ",camera_info_.visible_lenght_x_in_m_, camera_info_.visible_lenght_y_in_m_ , camera_info_.offset_cam_x_to_groundplane_, camera_info_.angle_horizontal_to_opening_);
	logger->log_debug(name(), "cam transform X: %f y: %f z: %f pitch: %f",camera_info_.position_x_, camera_info_.position_y_, camera_info_.position_z_, camera_info_.position_pitch_);
	loop();
}

void
PuckVisionThread::init()
{
	logger->log_info(name(), "starts init");

	config->add_change_handler(this);

	createPuckInterface();

	no_pucK_ = new puck();
	no_pucK_->visibiity_history = -9999;
	drawer_ = new firevision::FilterROIDraw();
	loadConfig();

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

//fawkes::PuckVisionInterface::PuckColor
//PuckVisionThread::getPuckInterfaceColor(firevision::ROI* roi){
//	switch (roi->color) {
//		case firevision::C_BLUE:
//			return fawkes::PuckVisionInterface::C_BLUE;
//		case firevision::C_RED:
//			return fawkes::PuckVisionInterface::C_RED;
//		case firevision::C_GREEN:
//			return fawkes::PuckVisionInterface::C_GREEN;
//		case firevision::C_YELLOW:
//			return fawkes::PuckVisionInterface::C_YELLOW;
//		case firevision::C_BLACK:
//			return fawkes::PuckVisionInterface::C_BLACK;
//		case firevision::C_WHITE:
//			return fawkes::PuckVisionInterface::C_WHITE;
//		default:
//			return fawkes::PuckVisionInterface::C_UNKNOWN;
//	}
//}

//void checkPadding(std::list<firevision::ROI>& dst,
//			std::list<firevision::ROI> src){
//	while(src.size() !=0 ) {
//
//	}
//}

void
PuckVisionThread::loop()
{
	if(!cfg_mutex_.try_lock()){
		logger->log_info(name(), "Skipping loop(), mutex locked");
		return; //If I cant lock it its locked already (config is changing/reinit in progress)
	}
	else{
		if(cam_ != NULL && !cam_->ready()){
				logger->log_info(name(), "camera not ready\n");
				loadConfig();
				return;
		}
		logger->log_info(name(), "capture");
		cam_->capture();
		firevision::convert(cspaceFrom_,
					cspaceTo_,
					cam_->buffer(),
					buffer_,
					camera_info_.img_width_,
					camera_info_.img_height_);
		cam_->dispose_buffer();


		std::list<firevision::ROI> rois_all_;

		if (cfg_colormodel_mode_ == "colormap"){

			scanline_->reset();
			classifier_red_->set_src_buffer(buffer_, camera_info_.img_width_, camera_info_.img_height_);
			std::list<firevision::ROI> *rois_red_ = classifier_red_->classify();
			mergeWithColorInformation(firevision::C_RED, rois_red_, &rois_all_);

			delete rois_red_;

		} else if(cfg_colormodel_mode_ == "similarity"){
			scanline_->reset();
			classifier_similarity_->set_src_buffer(buffer_, camera_info_.img_width_, camera_info_.img_height_);
			std::list<firevision::ROI> *rois_similarity_ = classifier_similarity_->classify();

			mergeWithColorInformation(firevision::C_MAGENTA, rois_similarity_, &rois_all_);

			delete rois_similarity_;
		}

		//cut to big rois


		//

		if(cfg_paintROIsActivated_){
				drawRois(&rois_all_);
		}

		//calculate ROI to POS3D
		std::list<puck> pucKs;

		//updateInterface(&list);
	}
	cfg_mutex_.unlock();
}

void detectFeature(){
//	scanline_->reset();
//	classifier_yellow_->set_src_buffer(buffer_, camera_info_.img_width_, camera_info_.img_height_);
//	std::list<firevision::ROI> *rois_yellow_ = classifier_yellow_->classify();
//
//	scanline_->reset();
//	classifier_green_->set_src_buffer(buffer_, camera_info_.img_width_, camera_info_.img_height_);
//	std::list<firevision::ROI> *rois_green_ = classifier_green_->classify();
//
//	scanline_->reset();
//	classifier_blue_->set_src_buffer(buffer_, camera_info_.img_width_, camera_info_.img_height_);
//	std::list<firevision::ROI> *rois_blue_ = classifier_blue_->classify();
//
//	mergeWithColorInformation(firevision::C_BLUE, rois_blue_, rois_all_);
//	mergeWithColorInformation(firevision::C_GREEN, rois_green_, rois_all_);
//	mergeWithColorInformation(firevision::C_YELLOW, rois_yellow_, rois_all_);
//
//	delete rois_green_;
//	delete rois_yellow_;
//	delete rois_blue_;
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
	drawer_->set_src_buffer(buffer_, firevision::ROI::full_image(camera_info_.img_width_, camera_info_.img_height_), 0);
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

	classifier->set_src_buffer(buffer_, camera_info_.img_width_, camera_info_.img_height_);

	std::list<firevision::ROI> *ROIs = classifier->classify();

	return ROIs;
}

float
PuckVisionThread::getX(firevision::ROI* roi){
	// Distance X
	int roiPositionY = camera_info_.img_height_ - ( roi->start.y +roi->height/2 );
	float angle = ( (float)roiPositionY * camera_info_.opening_angle_vertical_ ) / (float)camera_info_.img_height_;
	float distance_x = camera_info_.position_z_ * (tan( angle + camera_info_.angle_horizontal_to_opening_) - tan(camera_info_.angle_horizontal_to_opening_));

	//logger->log_debug(name(),"angle: %f, image_size x: %i y: %i,roi x: %i y: %i", (angle*180) / M_PI, img_width_, img_height_, roi->start.x, roi->start.y);

	return distance_x + camera_info_.offset_cam_x_to_groundplane_ + camera_info_.position_x_;
}

float
PuckVisionThread::getY(firevision::ROI* roi){
	// Left-Right
	int roiPositionX = ( roi->start.x + roi->width/2 );
	float alpha = ((((float)camera_info_.img_width_/2) - (float)roiPositionX) / ((float)camera_info_.img_width_ /2 )) * (camera_info_.opening_angle_horizontal_/2);
	float x = getX(roi);
	float position_y_in_m = sin(alpha * x);

	//logger->log_debug(name(),"angle: %f, roi pos X: %i, distance y: %f, cfg %f", alpha, roiPositionX, position_y_in_m, cfg_camera_opening_angle_horizontal_);

	return position_y_in_m;
}

//int
//PuckVisionThread::getVisibilityHistory(fawkes::polar_coord_2d_t polar,
//		fawkes::PuckVisionInterface::PuckColor colorRoi,
//		fawkes::PuckVisionInterface::PuckColor colorInterface,
//		float interface_phi, float interface_r,
//		int interface_visibility)
//{
//	bool toBigRotation = polar.phi - interface_phi > 0.1;
//	bool toBigMovement = polar.r - interface_r > 0.1;
//	bool colorChanged = colorRoi != colorInterface;
//	if (toBigRotation || toBigMovement || colorChanged
//			|| colorRoi == fawkes::PuckVisionInterface::C_UNKNOWN) {
//		return -1;
//	} else {
//		return ++interface_visibility;
//	}
//}

void
PuckVisionThread::updatePos3dInferface(fawkes::Position3DInterface* interface, puck* p){
	interface->set_translation(0, p->x);
	interface->set_translation(1, p->y);
	interface->set_translation(2, p->z);
	interface->set_visibility_history(p->visibiity_history);
}

void
PuckVisionThread::updateInterface(std::list<puck>* pucks){
	/* Create missing interfaces */
	if(pucks->size() > puck_interfaces_.size()){
		unsigned int nr_missing_interfaces = pucks->size() - puck_interfaces_.size();
		for(unsigned int i = 0; i < nr_missing_interfaces; ++i){
			createPuckInterface();
		}
	}

	if(cfg_debugMessagesActivated_) {
		logger->log_info(name(),"Detected pucks in view:  %i", pucks->size());
	}

	std::vector<fawkes::Position3DInterface*>::iterator interface_it = puck_interfaces_.begin();
	std::list<puck>::iterator puck_it = pucks->begin();

	for (; interface_it != puck_interfaces_.end(); ++interface_it) {
		fawkes::Position3DInterface* interface = (fawkes::Position3DInterface*)&interface_it;
		if(puck_it != pucks->end()){
			++puck_it;
			puck* p = (puck*)&puck_it;
			updatePos3dInferface(interface, p);
			interface->write();
		}
		else{
			updatePos3dInferface(interface, no_pucK_);
			interface->write();
		}
	}
}


void
sortByDistance(){
//	if (! cluster_indices.empty()) {
//	    std::vector<ClusterInfo> cinfos;
//
//	    for (unsigned int i = 0; i < cluster_indices.size(); ++i) {
//	      Eigen::Vector4f centroid;
//	      pcl::compute3DCentroid(*noline_cloud, cluster_indices[i].indices, centroid);
//	      if ( (centroid.x() >= cfg_cluster_min_x_) && (centroid.x() <= cfg_cluster_max_x_) &&
//		   (centroid.y() >= cfg_cluster_min_y_) && (centroid.y() <= cfg_cluster_max_y_))
//	      {
//		ClusterInfo info;
//		info.angle = std::atan2(centroid.y(), centroid.x());
//		info.dist  = centroid.norm();
//		info.index = i;
//		info.centroid = centroid;
//		cinfos.push_back(info);
//	      } else {
//		/*
//		logger->log_info(name(), "[L %u] Cluster %u out of bounds (%f,%f) "
//				 "not in ((%f,%f),(%f,%f))",
//				 loop_count_, centroid.x(), centroid.y(),
//				 cfg_cluster_min_x_, cfg_cluster_max_x_,
//				 cfg_cluster_min_y_, cfg_cluster_max_y_);
//		*/
//	      }
//	    }
//
//	    if (! cinfos.empty()) {
//	      if (cfg_selection_mode_ == SELECT_MIN_ANGLE) {
//		std::sort(cinfos.begin(), cinfos.end(),
//			  [](const ClusterInfo & a, const ClusterInfo & b) -> bool
//			  {
//			    return a.angle < b.angle;
//			  });
//	      } else if (cfg_selection_mode_ == SELECT_MIN_DIST) {
//		std::sort(cinfos.begin(), cinfos.end(),
//			  [](const ClusterInfo & a, const ClusterInfo & b) -> bool
//			  {
//			    return a.dist < b.dist;
//			  });
//	      } else {
//		logger->log_error(name(), "Invalid selection mode, cannot select cluster");
//	      }
}

void
PuckVisionThread::finalize()													//TODO check if everthing gets deleted
{
	logger->log_debug(name(), "finalize starts");

	vision_master->unregister_thread(this);
	delete shm_buffer_;
	config->rem_change_handler(this);
	delete cam_;
	cam_ = NULL;

	fawkes::Position3DInterface* interface;
	while(!puck_interfaces_.empty()){
		interface = puck_interfaces_.back();
		blackboard->close(interface);
		puck_interfaces_.pop_back();
	}

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

	delete cls_red_.colormodel;

	logger->log_info(name(), "finalize ends");
}


void PuckVisionThread::config_value_erased(const char *path) {};
void PuckVisionThread::config_tag_changed(const char *new_tag) {};
void PuckVisionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v) {};
void PuckVisionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{
	cfg_mutex_.lock();
	cfg_changed_ = true;
	loadConfig();
	cfg_changed_ = false;
	cfg_mutex_.unlock();
}







