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

	switchInterface_ = NULL;
	no_pucK_ = NULL;

	puck_info_.main.classifier = NULL;
	puck_info_.main.colormodel = NULL;
	puck_info_.main.scanline_grid = NULL;

	puck_info_.top_dots.classifier = NULL;
	puck_info_.top_dots.colormodel = NULL;
	puck_info_.top_dots.scanline_grid = NULL;

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

	std::string interface_name = "puck_" + std::to_string(puck_interfaces_.size());
	logger->log_info(name(), "Creating new Position3DInterface %s", interface_name.c_str());

	try {
		puck_if_ = blackboard->open_for_writing<fawkes::Position3DInterface>(interface_name.c_str());
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
	cfg_prefix_static_transforms_ = config->get_string((cfg_prefix_ + "transform").c_str());

	cfg_frame_ = config->get_string((cfg_prefix_static_transforms_ + "child_frame").c_str());

	//load camera informations
	camera_info_.cfg_camera_ = config->get_string((cfg_prefix_ + "camera").c_str());
	camera_info_.position_x_ = config->get_float(( cfg_prefix_static_transforms_ + "trans_x").c_str());
	camera_info_.position_y_ =  config->get_float(( cfg_prefix_static_transforms_ + "trans_y").c_str());
	camera_info_.position_z_ =  config->get_float(( cfg_prefix_static_transforms_ + "trans_z").c_str());
	camera_info_.position_pitch_ =  config->get_float(( cfg_prefix_static_transforms_ + "rot_pitch").c_str());
	camera_info_.opening_angle_horizontal_ = config->get_float((cfg_prefix_ + "camera_opening_angle_horizontal").c_str());
	camera_info_.opening_angle_vertical_ = config->get_float((cfg_prefix_ + "camera_opening_angle_vertical").c_str());
	// Calculate visible Area
	camera_info_.angle_horizontal_to_opening_ = (1.57 - camera_info_.position_pitch_) - (camera_info_.opening_angle_vertical_ / 2) ;
	camera_info_.visible_lenght_x_in_m_ = camera_info_.position_z_ * (tan( camera_info_.opening_angle_vertical_ + camera_info_.angle_horizontal_to_opening_) - tan(camera_info_.angle_horizontal_to_opening_));
	camera_info_.offset_cam_x_to_groundplane_ = camera_info_.position_z_ * tan(camera_info_.angle_horizontal_to_opening_);
	camera_info_.visible_lenght_y_in_m_ = camera_info_.position_z_ * tan (camera_info_.opening_angle_horizontal_ /2 );

	logger->log_debug(name(),"Camera opening angles Horizontal: %f Vertical: %f", camera_info_.opening_angle_horizontal_, camera_info_.opening_angle_vertical_);

	cfg_debugMessagesActivated_ = config->get_bool((cfg_prefix_ + "show_debug_messages").c_str());
	cfg_paintROIsActivated_ = config->get_bool((cfg_prefix_ + "draw_rois").c_str());

	search_area.start.x = config->get_uint((cfg_prefix_ + "search_area/start/x").c_str());
	search_area.start.y = config->get_uint((cfg_prefix_ + "search_area/start/y").c_str());
	search_area.width = config->get_uint((cfg_prefix_ + "search_area/width").c_str());
	search_area.height = config->get_uint((cfg_prefix_ + "search_area/height").c_str());
	search_area.color = firevision::C_BLUE;

	//Config Value for the classifier mode
	cfg_colormodel_mode_ = config->get_string((cfg_prefix_ + "colormodel_mode").c_str());

}

void PuckVisionThread::init_with_config()
{
	// CAM swapping not working
	if(false && cam_ != NULL){
		cam_->stop();
		cam_->flush();
		cam_->dispose_buffer();
		cam_->close();
		delete cam_;
		cam_ = NULL;
	}
	if(cam_ == NULL){
		cam_ = vision_master->register_for_camera(camera_info_.cfg_camera_.c_str(), this);
		cam_->start();
		cam_->open();
		camera_info_.img_width_ = cam_->pixel_width();
		camera_info_.img_height_ = cam_->pixel_height();

		search_area.image_height = camera_info_.img_height_;
		search_area.image_width = camera_info_.img_width_;
		printRoi(search_area);

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
		camera_info_.fullimage = firevision::ROI::full_image(camera_info_.img_width_, camera_info_.img_height_);
	}

//	//load puck infos;
	puck_info_.radius = config->get_float((cfg_prefix_ + "puck/radius").c_str());
	puck_info_.height = config->get_float((cfg_prefix_ + "puck/height").c_str());

	//Configure classifier
	logger->log_info(name(),"Loading colormodel %s", cfg_colormodel_mode_.c_str());

	setup_color_classifier(&puck_info_.main, "puck/red", firevision::C_RED);
	setup_color_classifier(&puck_info_.top_dots, "puck/topdots", firevision::C_YELLOW);

	logger->log_debug(name(), "Visible X: %f y: %f Offset cam groundplane: %f angle (horizontal/opening): %f ",camera_info_.visible_lenght_x_in_m_, camera_info_.visible_lenght_y_in_m_ , camera_info_.offset_cam_x_to_groundplane_, camera_info_.angle_horizontal_to_opening_);
	logger->log_debug(name(), "cam transform X: %f y: %f z: %f pitch: %f",camera_info_.position_x_, camera_info_.position_y_, camera_info_.position_z_, camera_info_.position_pitch_);
}

void
PuckVisionThread::init()
{
	logger->log_info(name(), "starts init");

	config->add_change_handler(this);

	no_pucK_ = new puck();
	int val_empty = -9999;
	no_pucK_->visibiity_history = val_empty;
	no_pucK_->cart.x = val_empty;
	no_pucK_->cart.y = val_empty;
	no_pucK_->cart.z = val_empty;
	drawer_ = new firevision::FilterROIDraw();
	loadConfig();
	init_with_config();
	createPuckInterface();

	logger->log_debug(name(), "end of init()");
}

void PuckVisionThread::deleteClassifier(
		color_classifier_context_t_* color_data) {
	if (color_data->classifier != NULL) {
		delete color_data->classifier;
		color_data->classifier = NULL;
	}
	if (color_data->colormodel != NULL) {
		delete color_data->colormodel;
		color_data->colormodel = NULL;
	}
	if (color_data->classifier != NULL) {
		delete color_data->scanline_grid;
		color_data->scanline_grid = NULL;
	}
	if( !color_data->color_classes.empty()){
		while(!color_data->color_classes.empty()){
				firevision::ColorModelSimilarity::color_class_t* color_class = color_data->color_classes.back();
				color_data->color_classes.pop_back();
				delete color_class;
		}
	}
}

void PuckVisionThread::setup_color_classifier(color_classifier_context_t_ *color_data, const char* prefix, firevision::color_t expected)
{
  deleteClassifier(color_data);
  //Update values from config
  color_data->color_expect = expected;
  color_data->cfg_ref_col = config->get_uints((cfg_prefix_ + prefix +"/reference_color").c_str());
  color_data->cfg_chroma_thresh = config->get_int((cfg_prefix_ + prefix +"/chroma_thresh").c_str());
  color_data->cfg_sat_thresh = config->get_int((cfg_prefix_ + prefix +"/saturation_thresh").c_str());
  color_data->cfg_roi_min_points = config->get_int((cfg_prefix_ + prefix +"/min_points").c_str());
  color_data->cfg_roi_basic_size = config->get_int((cfg_prefix_ + prefix +"/basic_roi_size").c_str());
  color_data->cfg_roi_neighborhood_min_match = config->get_int((cfg_prefix_ + prefix +"/neighborhood_min_match").c_str());
  color_data->cfg_scangrid_x_offset = config->get_int((cfg_prefix_ + prefix +"/scangrid_x_offset").c_str());
  color_data->cfg_scangrid_y_offset = config->get_int((cfg_prefix_ + prefix +"/scangrid_y_offset").c_str());

  color_data->scanline_grid = new firevision::ScanlineGrid(
 	camera_info_.img_width_,
 	camera_info_.img_height_,
    color_data->cfg_scangrid_x_offset,
    color_data->cfg_scangrid_y_offset);

  if (cfg_colormodel_mode_ == "colormap"){
	  std::string lutname =  std::string(prefix) + "/lut"; //"puck_vison/" +
	  std::string filename = std::string(CONFDIR) + "/"+ config->get_string((cfg_prefix_ + prefix +"/lut").c_str());
	  color_data->colormodel = new firevision::ColorModelLookupTable(filename.c_str(),
			  lutname.c_str(),
			  true /* destroy on delete */);

	  color_data->classifier = new firevision::SimpleColorClassifier(
	 	  color_data->scanline_grid,
	 	  color_data->colormodel,
	 	  color_data->cfg_roi_min_points,
	 	  color_data->cfg_roi_basic_size,
	 	  false,
	 	  color_data->cfg_roi_neighborhood_min_match,
	 	  0);

  } else if(cfg_colormodel_mode_ == "similarity"){
	  firevision::ColorModelSimilarity* cm = new firevision::ColorModelSimilarity();

	  while(!color_data->cfg_ref_col.empty()){
		  std::vector<unsigned int> color(color_data->cfg_ref_col.begin(), color_data->cfg_ref_col.begin() + 3);
		  color_data->cfg_ref_col = std::vector<unsigned int>(color_data->cfg_ref_col.begin() + 3, color_data->cfg_ref_col.end());
		  firevision::ColorModelSimilarity::color_class_t* color_class = new firevision::ColorModelSimilarity::color_class_t(
				  color_data->color_expect,
				  color,
				  color_data->cfg_chroma_thresh,
				  color_data->cfg_sat_thresh
				  );

		  color_data->color_classes.push_back(color_class);
		  cm->add_color(color_class);
		  logger->log_info(name(),"Color %i added reference color R:%i G:%i B:%i", color_data->color_expect, color[0], color[1], color[2] );
	  }

	color_data->colormodel = cm;

    color_data->classifier = new firevision::SimpleColorClassifier(
	  color_data->scanline_grid,
	  color_data->colormodel,
	  color_data->cfg_roi_min_points,
	  color_data->cfg_roi_basic_size,
	  false,
	  color_data->cfg_roi_neighborhood_min_match,
	  0,
	  color_data->color_expect);
  }
  else{
	throw new fawkes::Exception("selected colormodel not supported");
  }
}

void
PuckVisionThread::drawRois(std::list<firevision::ROI>* rois_) {
	if(cfg_paintROIsActivated_){
		for(std::list<firevision::ROI>::iterator list_iter = rois_->begin();
			list_iter != rois_->end(); list_iter++)
		{
			drawROIIntoBuffer(*list_iter, firevision::FilterROIDraw::DASHED_HINT);
		}
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
		//logger->log_info(name(), "capture");
		cam_->capture();
		firevision::convert(cspaceFrom_,
					cspaceTo_,
					cam_->buffer(),
					buffer_,
					camera_info_.img_width_,
					camera_info_.img_height_);
		cam_->dispose_buffer();

		std::list<firevision::ROI> pucks_in_view = detectPucks();
		std::vector<puck> pucks;

		//calculate ROI to POS3D
		calculatePuckPositions(&pucks, pucks_in_view);
		//sort pucks
		sortPucks(&pucks);

		updateInterface(&pucks);
	}
	cfg_mutex_.unlock();
}

void PuckVisionThread::sortPucks(std::vector<puck> *pucks){
	// Sort pucks
	std::sort(pucks->begin(), pucks->end(),
	[](const puck a, const puck & b) -> bool
	{
		return a.pol.r < b.pol.r;
	});

}

void PuckVisionThread::calculatePuckPositions(std::vector<puck> *pucks, std::list<firevision::ROI> pucks_in_view){
	for(std::list<firevision::ROI>::iterator it = pucks_in_view.begin();
		    it != pucks_in_view.end(); it++)
	{
		puck p;
		getPuckPosition(&p, (*it));
		pucks->push_back(p);
	}
}

void PuckVisionThread::printRoi(firevision::ROI roi){
	std::printf("x: %i y: %i width: %i height: %i color: %i image_hight: %i image_width: %i \n",
			roi.start.x,
			roi.start.y,
			roi.width,
			roi.height,
			roi.color,
			roi.image_height,
			roi.image_width);
}

std::list<firevision::ROI>* splitROI(firevision::ROI bigroi, firevision::ROI cut){
	std::list<firevision::ROI>* rois = new std::list<firevision::ROI>();

//	unsigned int center_x = cut.start.x+cut.width/2;
//	unsigned int center_y = cut.start.y+cut.height/2;
//	if(bigroi.contains(center_x, center_y)){
//		if( (bigroi.width - cut.width) > 50){ //check if roi width makes sense to cut
//
//			firevision::ROI roi(bigroi);
//			int posx = bigroi.start.x - cut.start.x;
//			if( posx > 0 ){ //cut left from roi
//				//change width
//			}
//			else{
//				//change width and start
//
//			}
//		}
//	}

	return rois;
}

void fitROI(firevision::ROI &roi, int x, int y, int w, int h){
  //Check x position in in image range
  if(x < 0) {roi.start.x = 0; w = w+x; }
  else if(x > (int)roi.image_width){ roi.start.x = roi.image_width-1; }
  else{ roi.start.x = x; }

  //Check y position in in image range
  if(y < 0) {roi.start.y = 0; h = h+y; }
  else if(y > (int)roi.image_height){ roi.start.y = roi.image_height-1; }
  else{ roi.start.y = y; }

  //Check width+start.x is in image range
  if( w < 0){ roi.width = 0; }
  else if(x + w > (int)roi.image_width){ roi.width = roi.image_width - roi.start.x; }
  else{ roi.width = w; }

  //Check height+start.y is in image range
  if(h < 0) { roi.height = 0; }
  else if(y + h > (int)roi.image_height){ roi.height = roi.image_height - roi.start.y; }
  else{ roi.height = h; }
}

/** detectPucks
 *  Returns a list of ROIs. Each ROI contians a puck
 */
std::list<firevision::ROI> PuckVisionThread::detectPucks(){

	std::list<firevision::ROI> rois_all_;
	std::list<firevision::ROI> pucks;

	std::list<firevision::ROI> *rois_red_ = classifyInRoi(search_area, &puck_info_.main);

	for (std::list<firevision::ROI>::iterator it = rois_red_->begin();
			it != rois_red_->end(); ++it) {
		unsigned int resize = it->height/10;
		it->height += resize; // yellow is usually on top of the pucks
		it->start.y-= resize; // Move roi up after enlarging
		firevision::ROI possible_puck = (*it);
		std::list<firevision::ROI>* yellow_rois = classifyInRoi(possible_puck, &puck_info_.top_dots);
		drawRois(yellow_rois);
		if(yellow_rois->size() < 2){
			//rois_red_->remove(*it); // Not enough features
			//it = rois_red_->begin(); // not nice...

			//logger->log_info(name(), "no yellow");

			//Should we add possible pucks?
			//pucks.push_back(possible_puck);
		}
		else{
			//find yellow roi with lowest left corner;
			//x min
			//y max
			firevision::ROI lowest_yellow_dot_in_roi = yellow_rois->front();

			for (std::list<firevision::ROI>::iterator it = yellow_rois->begin();
						it != yellow_rois->end(); ++it) {
				//unsigned int center_x = (*it).start.x + (*it).width/2;

				if((*it).start.y > lowest_yellow_dot_in_roi.start.y){
					lowest_yellow_dot_in_roi = (*it);
				}
//				if(center_x < lowest_yellow_dot_in_roi.start.x + lowest_yellow_dot_in_roi.width/2){
//
//				}
			}
			// now we have the lowest yellow dot in the roi.
			lowest_yellow_dot_in_roi.color = firevision::C_CYAN;
			drawROIIntoBuffer(lowest_yellow_dot_in_roi);

			// create a new smaller roi
			int x,y,w,h;
			h = possible_puck.height + possible_puck.start.y - lowest_yellow_dot_in_roi.start.y + lowest_yellow_dot_in_roi.height;//6*lowest_yellow_dot_in_roi.height;
			//w = lowest_yellow_dot_in_roi.image_width;
			w = 16*lowest_yellow_dot_in_roi.width;
			//x = 0;
			x = lowest_yellow_dot_in_roi.start.x - w/2;
			y = lowest_yellow_dot_in_roi.start.y-lowest_yellow_dot_in_roi.height;// + lowest_yellow_dot_in_roi.height;

			firevision::ROI look_for_puck(lowest_yellow_dot_in_roi);
			fitROI(look_for_puck,x,y,w,h);
			look_for_puck.color = firevision::C_ORANGE;

			drawROIIntoBuffer(look_for_puck);

			std::list<firevision::ROI> *pucks_with_feature = classifyInRoi(look_for_puck, &puck_info_.main);
			// Get biggest roi, this ist the puck
			if(pucks_with_feature->size() > 0){
				firevision::ROI puck = getRoiContainingRoi(pucks_with_feature, lowest_yellow_dot_in_roi);
				puck.color = firevision::C_WHITE;
				pucks.push_back(puck);


				//TODO Split rois to find more than one puck per ROI
				//std::list<firevision::ROI>* splitted_roi = splitROI(possible_puck, puck);

				//rois_red_->merge(*splitted_roi);
				mergeWithColorInformation(firevision::C_BLACK, pucks_with_feature, &rois_all_ );
			}
			delete pucks_with_feature;
		}
		delete yellow_rois;
	}
	mergeWithColorInformation(puck_info_.main.color_expect, rois_red_, &rois_all_);
	//logger->log_info(name(), "Pucks %i colormodel %s", pucks.size(), puck_info_.main.colormodel->get_name());

	drawRois(&rois_all_);
	drawRois(&pucks);
	drawROIIntoBuffer(search_area);

	return pucks;
}

firevision::ROI PuckVisionThread::getRoiContainingRoi( std::list<firevision::ROI>* roiList, firevision::ROI containing) {
	for (std::list<firevision::ROI>::iterator it = roiList->begin();
			it != roiList->end(); ++it) {
		if ((*it).contains(containing.start.x + containing.width/2, + containing.start.y + containing.height/2)) {
			return (*it);
		}
	}
	return getBiggestRoi(roiList);
}

firevision::ROI PuckVisionThread::getBiggestRoi( std::list<firevision::ROI>* roiList) {
	firevision::ROI biggestRoi;

	for (std::list<firevision::ROI>::iterator it = roiList->begin();
			it != roiList->end(); ++it) {
		if (biggestRoi.width * biggestRoi.height
				< (*it).width * (*it).height) {
			biggestRoi = (*it);
		}
	}
	return biggestRoi;
}

PuckVisionThread::~PuckVisionThread()
{
	// TODO Auto-generated destructor stub
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
	if(cfg_paintROIsActivated_){
		drawer_->set_src_buffer(buffer_, firevision::ROI::full_image(camera_info_.img_width_, camera_info_.img_height_), 0);
		drawer_->set_dst_buffer(buffer_, &roi);
		drawer_->set_style(borderStyle);
		drawer_->apply();

		if (cfg_debugMessagesActivated_) {
			logger->log_debug(name(), "drawed element in buffer");
		}
	}
}


std::list<firevision::ROI>*
PuckVisionThread::classifyInRoi(firevision::ROI searchArea, color_classifier_context_t_* color_data)
{
	color_data->scanline_grid->reset();
	color_data->scanline_grid->set_roi(&searchArea);
	color_data->classifier->set_src_buffer(buffer_, camera_info_.img_width_, camera_info_.img_height_);

	std::list<firevision::ROI> *ROIs = color_data->classifier->classify();

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

void
PuckVisionThread::getPuckPosition(puck *p, firevision::ROI roi){
	// Left-Right
	int roiPositionX_L = ( roi.start.x);
	int roiPositionX_R = ( roi.start.x + roi.width );
	float alpha_L = ((((float)camera_info_.img_width_/2) - (float)roiPositionX_L) / ((float)camera_info_.img_width_ /2 )) * (camera_info_.opening_angle_horizontal_/2);
	float alpha_R = ((((float)camera_info_.img_width_/2) - (float)roiPositionX_R) / ((float)camera_info_.img_width_ /2 )) * (camera_info_.opening_angle_horizontal_/2);
	float x = getX(&roi);
	float position_y_in_m_L = sin(alpha_L * x);
	float position_y_in_m_R = sin(alpha_R * x);

	//logger->log_debug(name(),"angle: %f, roi pos X: %i, distance y: %f, cfg %f", alpha, roiPositionX, position_y_in_m, cfg_camera_opening_angle_horizontal_);

	p->roi = roi;
	p->cart.x = getX(&roi);
	p->cart.y = getY(&roi);
	cartToPol(p->pol, p->cart.x, p->cart.y );
	p->visibiity_history = 1;
	p->radius = position_y_in_m_R - position_y_in_m_L;
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
	interface->set_translation(0, p->cart.x);
	interface->set_translation(1, p->cart.y);
	interface->set_translation(2, p->cart.z);
	interface->set_visibility_history(p->visibiity_history);
}

void
PuckVisionThread::updateInterface(std::vector<puck>* pucks){
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
	std::vector<puck>::iterator puck_it = pucks->begin();
	for (; interface_it != puck_interfaces_.end(); ++interface_it) {
		fawkes::Position3DInterface* interface = (*interface_it);
		if(puck_it != pucks->end()){
			updatePos3dInferface(interface, &(*puck_it));
			interface->write();
			++puck_it;
		}
		else{
			updatePos3dInferface(interface, no_pucK_);
			interface->write();
		}
	}
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

	deleteClassifier(&puck_info_.main);
	deleteClassifier(&puck_info_.top_dots);

	logger->log_info(name(), "finalize ends");
}


void PuckVisionThread::config_value_erased(const char *path) {};
void PuckVisionThread::config_tag_changed(const char *new_tag) {};
void PuckVisionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v) {};
void PuckVisionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{

	if(cfg_mutex_.try_lock()){
		try{
			loadConfig();
			init_with_config();
		}
		catch(fawkes::Exception &e){
			logger->log_error(name(), e);
		}
	}
	 // gets called for every changed entry... so init is called once per change.
	cfg_mutex_.unlock();
}







