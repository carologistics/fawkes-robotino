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
: Thread("PuckVisionThread", Thread::OPMODE_WAITFORWAKEUP),
  VisionAspect(VisionAspect::CYCLIC),
  ConfigurationChangeHandler(CFG_PREFIX)
//,   ConfigurationChangeHandler(CFG_TRANSFORM_PREFIX)
{
	cfg_prefix_ = CFG_PREFIX;

	camera_info_.connection               = "";
	camera_info_.img_width                = 0;
	camera_info_.img_height               = 0;
	camera_info_.opening_angle_horizontal = 0;
	camera_info_.opening_angle_vertical   = 0;

	buffer_     = NULL;
	shm_buffer_ = NULL;

	cspaceFrom_ = firevision::YUV422_PLANAR;
	cspaceTo_   = firevision::YUV422_PLANAR;

	cam_ = NULL;

	switchInterface_ = NULL;

	puck_info_.main.classifier    = NULL;
	puck_info_.main.colormodel    = NULL;
	puck_info_.main.scanline_grid = NULL;

	puck_info_.top_dots.classifier    = NULL;
	puck_info_.top_dots.colormodel    = NULL;
	puck_info_.top_dots.scanline_grid = NULL;

	drawer_ = NULL;

	cfg_paintROIsActivated_     = false;
	cfg_debugMessagesActivated_ = false;
	cfg_changed_                = false;
}
/*
 * Create a new puck interface for writing and append it to puck_interfaces_
 * vector
 */
void
PuckVisionThread::createPuckInterface()
{
	fawkes::Position3DInterface *puck_if_ = NULL;

	std::string interface_name = "puck_" + std::to_string(puck_interfaces_.size());
	logger->log_info(name(), "Creating new Position3DInterface %s", interface_name.c_str());

	try {
		puck_if_ = blackboard->open_for_writing<fawkes::Position3DInterface>(interface_name.c_str());
		puck_if_->set_frame(camera_info_.frame.c_str());
		puck_if_->write();
		puck_interfaces_.push_back(puck_if_);
	} catch (std::exception &e) {
		finalize();
		throw;
	}
}

void
PuckVisionThread::loadConfig()
{
	logger->log_info(name(), "loading config");
	cfg_prefix_                   = CFG_PREFIX;
	cfg_prefix_static_transforms_ = config->get_string((cfg_prefix_ + "transform").c_str());

	cfg_frame_target = "/base_link";

	cfg_use_new_pucks_ = config->get_bool((cfg_prefix_ + "use_new_pucks").c_str());

	// load camera informations
	camera_info_.connection = config->get_string((cfg_prefix_ + "camera").c_str());
	camera_info_.position_x = config->get_float((cfg_prefix_static_transforms_ + "trans_x").c_str());
	camera_info_.position_y = config->get_float((cfg_prefix_static_transforms_ + "trans_y").c_str());
	camera_info_.position_z = config->get_float((cfg_prefix_static_transforms_ + "trans_z").c_str());
	camera_info_.position_pitch =
	  config->get_float((cfg_prefix_static_transforms_ + "rot_pitch").c_str());
	camera_info_.opening_angle_horizontal =
	  config->get_float((cfg_prefix_ + "camera_opening_angle_horizontal").c_str());
	camera_info_.opening_angle_vertical =
	  config->get_float((cfg_prefix_ + "camera_opening_angle_vertical").c_str());
	// Calculate visible Area
	camera_info_.angle_horizontal_to_opening_ =
	  (1.57 - camera_info_.position_pitch) - (camera_info_.opening_angle_vertical / 2);
	camera_info_.visible_lenght_x_in_m_ =
	  camera_info_.position_z
	  * (tan(camera_info_.opening_angle_vertical + camera_info_.angle_horizontal_to_opening_)
	     - tan(camera_info_.angle_horizontal_to_opening_));
	camera_info_.offset_cam_x_to_groundplane_ =
	  camera_info_.position_z * tan(camera_info_.angle_horizontal_to_opening_);
	camera_info_.visible_lenght_y_in_m_ =
	  camera_info_.position_z * tan(camera_info_.opening_angle_horizontal / 2);
	camera_info_.frame = config->get_string((cfg_prefix_static_transforms_ + "child_frame").c_str());

	logger->log_debug(name(),
	                  "Camera opening angles Horizontal: %f Vertical: %f",
	                  camera_info_.opening_angle_horizontal,
	                  camera_info_.opening_angle_vertical);

	cfg_debugMessagesActivated_ = config->get_bool((cfg_prefix_ + "show_debug_messages").c_str());
	cfg_paintROIsActivated_     = config->get_bool((cfg_prefix_ + "draw_rois").c_str());

	search_area.start.x = config->get_uint((cfg_prefix_ + "search_area/start/x").c_str());
	search_area.start.y = config->get_uint((cfg_prefix_ + "search_area/start/y").c_str());
	search_area.width   = config->get_uint((cfg_prefix_ + "search_area/width").c_str());
	search_area.height  = config->get_uint((cfg_prefix_ + "search_area/height").c_str());
	search_area.color   = firevision::C_BLUE;

	// Config Value for the classifier mode
	cfg_check_yellow_dots_ = config->get_bool((cfg_prefix_ + "puck/dots_required").c_str());
}

void
PuckVisionThread::init_with_config()
{
	// CAM swapping not working
	if (false && cam_ != NULL) {
		cam_->stop();
		cam_->flush();
		cam_->dispose_buffer();
		cam_->close();
		delete cam_;
		cam_ = NULL;
	}
	if (cam_ == NULL) {
		cam_ = vision_master->register_for_camera(camera_info_.connection.c_str(), this);
		cam_->start();
		cam_->open();
		camera_info_.img_width  = cam_->pixel_width();
		camera_info_.img_height = cam_->pixel_height();

		search_area.image_height = camera_info_.img_height;
		search_area.image_width  = camera_info_.img_width;
		printRoi(search_area);

		cspaceFrom_ = cam_->colorspace();

		// SHM image buffer
		if (shm_buffer_ != NULL) {
			delete shm_buffer_;
			shm_buffer_ = NULL;
			buffer_     = NULL;
		}
		std::string shmID = config->get_string((cfg_prefix_ + "shm_image_id").c_str());
		shm_buffer_       = new firevision::SharedMemoryImageBuffer(shmID.c_str(),
                                                          cspaceTo_,
                                                          camera_info_.img_width,
                                                          camera_info_.img_height);

		if (!shm_buffer_->is_valid()) {
			delete shm_buffer_;
			delete cam_;
			shm_buffer_ = NULL;
			cam_        = NULL;
			throw fawkes::Exception("Shared memory segment not valid");
		}
		shm_buffer_->set_frame_id(camera_info_.frame.c_str());

		buffer_ = shm_buffer_->buffer();
		camera_info_.fullimage =
		  firevision::ROI::full_image(camera_info_.img_width, camera_info_.img_height);
	}

	//	//load puck infos;
	puck_info_.radius = config->get_float((cfg_prefix_ + "puck/radius").c_str());
	puck_info_.height = config->get_float((cfg_prefix_ + "puck/height").c_str());

	firevision::color_t color_top_dots;
	std::string         prefix_new_pucks = "";
	if (cfg_use_new_pucks_) {
		color_top_dots   = firevision::C_BLACK;
		prefix_new_pucks = "/new_pucks";
	} else {
		color_top_dots = firevision::C_YELLOW;
	}

	setup_color_classifier(&puck_info_.main,
	                       ("puck/red" + prefix_new_pucks).c_str(),
	                       firevision::C_RED);
	setup_color_classifier(&puck_info_.top_dots,
	                       ("puck/topdots" + prefix_new_pucks).c_str(),
	                       color_top_dots);

	logger->log_debug(name(),
	                  "Visible X: %f y: %f Offset cam groundplane: %f angle "
	                  "(horizontal/opening): %f ",
	                  camera_info_.visible_lenght_x_in_m_,
	                  camera_info_.visible_lenght_y_in_m_,
	                  camera_info_.offset_cam_x_to_groundplane_,
	                  camera_info_.angle_horizontal_to_opening_);
	logger->log_debug(name(),
	                  "cam transform X: %f y: %f z: %f pitch: %f",
	                  camera_info_.position_x,
	                  camera_info_.position_y,
	                  camera_info_.position_z,
	                  camera_info_.position_pitch);
}

void
PuckVisionThread::init()
{
	logger->log_info(name(), "starts init");

	config->add_change_handler(this);

	drawer_ = new firevision::FilterROIDraw();
	loadConfig();
	init_with_config();
	createPuckInterface();

	logger->log_debug(name(), "end of init()");
}

void
PuckVisionThread::deleteClassifier(color_classifier_context_t_ *color_data)
{
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
	if (!color_data->color_classes.empty()) {
		while (!color_data->color_classes.empty()) {
			firevision::ColorModelSimilarity::color_class_t *color_class =
			  color_data->color_classes.back();
			color_data->color_classes.pop_back();
			delete color_class;
		}
	}
}

void
PuckVisionThread::setup_color_classifier(color_classifier_context_t_ *color_data,
                                         const char *                 prefix,
                                         firevision::color_t          expected)
{
	deleteClassifier(color_data);
	// Update values from config
	color_data->color_expect = expected;
	color_data->cfg_ref_col  = config->get_uints((cfg_prefix_ + prefix + "/reference_color").c_str());
	color_data->cfg_chroma_thresh =
	  config->get_int((cfg_prefix_ + prefix + "/chroma_thresh").c_str());
	color_data->cfg_sat_thresh =
	  config->get_int((cfg_prefix_ + prefix + "/saturation_thresh").c_str());
	color_data->cfg_roi_min_points = config->get_int((cfg_prefix_ + prefix + "/min_points").c_str());
	color_data->cfg_roi_basic_size =
	  config->get_int((cfg_prefix_ + prefix + "/basic_roi_size").c_str());
	color_data->cfg_roi_neighborhood_min_match =
	  config->get_int((cfg_prefix_ + prefix + "/neighborhood_min_match").c_str());
	color_data->cfg_scangrid_x_offset =
	  config->get_int((cfg_prefix_ + prefix + "/scangrid_x_offset").c_str());
	color_data->cfg_scangrid_y_offset =
	  config->get_int((cfg_prefix_ + prefix + "/scangrid_y_offset").c_str());

	std::string cfg_color_mode =
	  config->get_string((cfg_prefix_ + prefix + "/colormodel_mode").c_str());

	color_data->scanline_grid = new firevision::ScanlineGrid(camera_info_.img_width,
	                                                         camera_info_.img_height,
	                                                         color_data->cfg_scangrid_x_offset,
	                                                         color_data->cfg_scangrid_y_offset);

	if (cfg_color_mode == "colormap") {
		std::string lutname = std::string(prefix) + "/lut"; //"puck_vison/" +
		std::string filename =
		  std::string(CONFDIR) + "/" + config->get_string((cfg_prefix_ + prefix + "/lut").c_str());
		color_data->colormodel = new firevision::ColorModelLookupTable(filename.c_str(),
		                                                               lutname.c_str(),
		                                                               true /* destroy on delete */
		);

	} else if (cfg_color_mode == "similarity") {
		firevision::ColorModelSimilarity *cm = new firevision::ColorModelSimilarity();

		while (!color_data->cfg_ref_col.empty()) {
			std::vector<unsigned int> color(color_data->cfg_ref_col.begin(),
			                                color_data->cfg_ref_col.begin() + 3);
			color_data->cfg_ref_col = std::vector<unsigned int>(color_data->cfg_ref_col.begin() + 3,
			                                                    color_data->cfg_ref_col.end());
			firevision::ColorModelSimilarity::color_class_t *color_class =
			  new firevision::ColorModelSimilarity::color_class_t(color_data->color_expect,
			                                                      color,
			                                                      color_data->cfg_chroma_thresh,
			                                                      color_data->cfg_sat_thresh);

			color_data->color_classes.push_back(color_class);
			cm->add_color(color_class);
			logger->log_info(name(),
			                 "Color %i added reference color R:%i G:%i B:%i",
			                 color_data->color_expect,
			                 color[0],
			                 color[1],
			                 color[2]);
		}

		color_data->colormodel = cm;
		color_data->classifier =
		  new firevision::SimpleColorClassifier(color_data->scanline_grid,
		                                        color_data->colormodel,
		                                        color_data->cfg_roi_min_points,
		                                        color_data->cfg_roi_basic_size,
		                                        false,
		                                        color_data->cfg_roi_neighborhood_min_match,
		                                        0,
		                                        color_data->color_expect);
	} else if (cfg_color_mode == "thresholds_black") {
		//	  unsigned int y_thresh;
		//	  unsigned int u_thresh;
		//	  unsigned int v_thresh;
		//	  unsigned int ref_u;
		//	  unsigned int ref_v;
		firevision::ColorModelBlack *cm = new firevision::ColorModelBlack(
		  /*y_thresh, u_thresh, v_thresh, ref_u, ref_v*/);

		color_data->colormodel = cm;
		color_data->classifier =
		  new firevision::SimpleColorClassifier(color_data->scanline_grid,
		                                        color_data->colormodel,
		                                        color_data->cfg_roi_min_points,
		                                        color_data->cfg_roi_basic_size,
		                                        false,
		                                        color_data->cfg_roi_neighborhood_min_match,
		                                        0,
		                                        color_data->color_expect);
	} else {
		throw new fawkes::Exception("selected colormodel not supported");
	}
}

void
PuckVisionThread::drawRois(std::list<firevision::ROI> *rois_)
{
	if (cfg_paintROIsActivated_) {
		for (std::list<firevision::ROI>::iterator list_iter = rois_->begin(); list_iter != rois_->end();
		     list_iter++) {
			drawROIIntoBuffer(*list_iter, firevision::FilterROIDraw::DASHED_HINT);
		}
	}
}

void
PuckVisionThread::mergeWithColorInformation(firevision::color_t         color,
                                            std::list<firevision::ROI> *rois_color_,
                                            std::list<firevision::ROI> *rois_all)
{
	while (rois_color_->size() != 0) {
		rois_color_->front().color = color;
		rois_all->push_front(rois_color_->front());
		rois_color_->pop_front();
	}
}

void
PuckVisionThread::loop()
{
	if (!cfg_mutex_.try_lock()) {
		logger->log_info(name(), "Skipping loop(), mutex locked");
		return; // If I cant lock it its locked already (config is changing/reinit
		        // in progress)
	} else {
		if (cam_ != NULL && !cam_->ready()) {
			logger->log_info(name(), "camera not ready\n");
			loadConfig();
			return;
		}
		// logger->log_info(name(), "capture");
		cam_->capture();
		firevision::convert(cspaceFrom_,
		                    cspaceTo_,
		                    cam_->buffer(),
		                    buffer_,
		                    camera_info_.img_width,
		                    camera_info_.img_height);
		cam_->dispose_buffer();

		std::list<firevision::ROI> pucks_in_view = detectPucks();
		std::vector<puck>          pucks;

		// calculate ROI to POS3D
		calculatePuckPositions(&pucks, pucks_in_view);
		// sort pucks
		sortPucks(&pucks);

		updateInterface(&pucks);
	}
	cfg_mutex_.unlock();
}

void
PuckVisionThread::sortPucks(std::vector<puck> *pucks)
{
	// Sort pucks
	std::sort(pucks->begin(), pucks->end(), [](const puck a, const puck &b) -> bool {
		return a.pol.r < b.pol.r;
	});
}

void
PuckVisionThread::calculatePuckPositions(std::vector<puck> *        pucks,
                                         std::list<firevision::ROI> pucks_in_view)
{
	for (std::list<firevision::ROI>::iterator it = pucks_in_view.begin(); it != pucks_in_view.end();
	     it++) {
		puck p;
		getPuckPosition(&p, (*it));
		pucks->push_back(p);
	}
}

void
PuckVisionThread::printRoi(firevision::ROI roi)
{
	if (cfg_debugMessagesActivated_) {
		logger->log_debug(name(),
		                  "x: %i y: %i width: %i height: %i color: %i image_hight: "
		                  "%i image_width: %i",
		                  roi.start.x,
		                  roi.start.y,
		                  roi.width,
		                  roi.height,
		                  roi.color,
		                  roi.image_height,
		                  roi.image_width);
	}
}

std::list<firevision::ROI> *
splitROI(firevision::ROI bigroi, firevision::ROI cut)
{
	std::list<firevision::ROI> *rois = new std::list<firevision::ROI>();

	//	unsigned int center_x = cut.start.x+cut.width/2;
	//	unsigned int center_y = cut.start.y+cut.height/2;
	//	if(bigroi.contains(center_x, center_y)){
	//		if( (bigroi.width - cut.width) > 50){ //check if roi width makes
	// sense to cut
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

void
PuckVisionThread::fitROI(firevision::ROI &roi, int x, int y, int w, int h)
{
	// Check x position in in image range
	if (cfg_debugMessagesActivated_) {
		logger->log_debug(name(), "Fit puck %i %i %i %i", x, y, w, h);
		printRoi(roi);
	}

	firevision::ROI old(roi);

	roi.start.x = std::max(0, (std::min((int)roi.image_width, x)));
	roi.start.y = std::max(0, (std::min((int)roi.image_height, y)));

	roi.width  = std::max(0, (std::min((int)roi.image_width, w)));
	roi.height = std::max(0, (std::min((int)roi.image_height, h)));

	if (roi.start.x + roi.width > roi.image_width) {
		roi.width = roi.image_width - roi.start.x;
	}

	if (roi.start.y + roi.height > roi.image_height) {
		roi.height = roi.image_height - roi.start.y;
	}

	if (roi.start.y + roi.height > roi.image_height || roi.start.x + roi.width > roi.image_width) {
		logger->log_error(name(),
		                  "Failed fitRoi(x: %i y: %i w: %i h: %i) with roi: x: %i y:%i width: %i "
		                  "height: %i , image_width: %i image_height: %i",
		                  x,
		                  y,
		                  w,
		                  h,
		                  old.start.x,
		                  old.start.y,
		                  old.width,
		                  old.height,
		                  old.image_width,
		                  old.image_height);
		logger->log_error(name(),
		                  "Results Roi: x: %i y:%i width: %i height: %i , "
		                  "image_width: %i image_height: %i",
		                  roi.start.x,
		                  roi.start.y,
		                  roi.width,
		                  roi.height,
		                  roi.image_width,
		                  roi.image_height);
	} else if (cfg_debugMessagesActivated_) {
		printRoi(roi);
	}
}

/** detectPucks
 *  Returns a list of ROIs. Each ROI contains a Puck
 */
std::list<firevision::ROI>
PuckVisionThread::detectPucks()
{
	std::list<firevision::ROI> rois_all_;
	std::list<firevision::ROI> pucks;

	// Find all RED Areas ( Can contain multiple Pucks, or no Pucks (just red
	// without any features)
	std::list<firevision::ROI> *rois_red_ = classifyInRoi(search_area, &puck_info_.main);

	for (std::list<firevision::ROI>::iterator big_red_rois_it = rois_red_->begin();
	     big_red_rois_it != rois_red_->end();
	     ++big_red_rois_it) {
		firevision::ROI scan_for_red((*big_red_rois_it));
		{
			// Scan the bottem of the red roi to seperate the pucks
			int x, y, w, h;
			h = std::max(search_area.height / 10, (unsigned int)10);
			w = (*big_red_rois_it).width; // scan the full width of the search area
			x = (*big_red_rois_it).start.x;
			y = (*big_red_rois_it).start.y + (*big_red_rois_it).height
			    - h / 2; // search in the lowest area of the possible puck

			fitROI(scan_for_red, x, y, w, h);
		}

		scan_for_red.color = firevision::C_GREEN;
		drawROIIntoBuffer(scan_for_red);

		std::list<firevision::ROI> *possible_pucks_bottem =
		  classifyInRoi(scan_for_red, &puck_info_.main);
		// Scan for red end

		// enlarge rois (height and check for puck features (Yellow dots)
		for (std::list<firevision::ROI>::iterator possible_puck_it = possible_pucks_bottem->begin();
		     possible_puck_it != possible_pucks_bottem->end();
		     ++possible_puck_it) {
			// Enlarge ROI for yellow dots
			if (cfg_check_yellow_dots_) {
				// unsigned int shift = possible_puck_it->height/10; //shift roi up a
				// bit (yellow dots can be over the red area)
				(*possible_puck_it).color = firevision::C_BLUE;
				drawROIIntoBuffer((*possible_puck_it));
				int x, y, w, h;
				int enlarge = (*possible_puck_it).width / 5;
				;
				h = (*possible_puck_it).width;
				x = (*possible_puck_it).start.x - enlarge / 2;
				y = (*possible_puck_it).start.y + (*possible_puck_it).height - h; // lower bound - hight
				w = (*possible_puck_it).width + enlarge;
				fitROI((*possible_puck_it), x, y, w, h);

				// Search in this roi for yellow dots
				std::list<firevision::ROI> *yellow_rois =
				  classifyInRoi((*possible_puck_it), &puck_info_.top_dots);
				drawRois(yellow_rois);
				if (yellow_rois->size() > 2) { // ROI has enough features its a PUCK
					firevision::ROI mypuck((*possible_puck_it));
					mypuck.color = firevision::C_WHITE;
					pucks.push_back(mypuck);
				}
				delete yellow_rois;
			} else { /* Dont check for yellow dots, all elements with proper color are
                  pucks */
				firevision::ROI mypuck((*possible_puck_it));
				mypuck.color = firevision::C_WHITE;
				pucks.push_back(mypuck);
			}
		}

		delete possible_pucks_bottem;
	}

	drawRois(&rois_all_);
	drawRois(&pucks);
	drawROIIntoBuffer(search_area);

	return pucks;
}

firevision::ROI *
PuckVisionThread::getRoiContainingRoi(std::list<firevision::ROI> *roiList,
                                      firevision::ROI             containing)
{
	for (std::list<firevision::ROI>::iterator it = roiList->begin(); it != roiList->end(); ++it) {
		if ((*it).contains(containing.start.x + containing.width / 2,
		                   +containing.start.y + containing.height / 2)) {
			return new firevision::ROI(*it);
		}
	}
	return NULL; // getBiggestRoi(roiList);
}

firevision::ROI *
PuckVisionThread::getBiggestRoi(std::list<firevision::ROI> *roiList)
{
	firevision::ROI *biggestRoi = NULL;

	for (std::list<firevision::ROI>::iterator it = roiList->begin(); it != roiList->end(); ++it) {
		if (biggestRoi == NULL || biggestRoi->width * biggestRoi->height < (*it).width * (*it).height) {
			biggestRoi = &(*it);
		}
	}
	if (biggestRoi != NULL) {
		return new firevision::ROI(biggestRoi);
	}
	return NULL;
}

PuckVisionThread::~PuckVisionThread()
{
	// TODO Auto-generated destructor stub
}

void
PuckVisionThread::cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y)
{
	pol.phi = atan2f(y, x);
	pol.r   = sqrtf(x * x + y * y);

	if (cfg_debugMessagesActivated_) {
		logger->log_debug(name(), "x: %f; y: %f", x, y);
		logger->log_debug(name(), "Calculated r: %f; phi: %f", pol.r, pol.phi);
	}
}

void
PuckVisionThread::polToCart(float &x, float &y, fawkes::polar_coord_2d_t pol)
{
	x = pol.r * std::cos(pol.phi);
	y = pol.r * std::sin(pol.phi);
}

void
PuckVisionThread::drawROIIntoBuffer(firevision::ROI                           roi,
                                    firevision::FilterROIDraw::border_style_t borderStyle)
{
	try {
		if (cfg_paintROIsActivated_) {
			drawer_->set_src_buffer(
			  buffer_, firevision::ROI::full_image(camera_info_.img_width, camera_info_.img_height), 0);
			drawer_->set_dst_buffer(buffer_, &roi);
			drawer_->set_style(borderStyle);
			drawer_->apply();
			if (cfg_debugMessagesActivated_) {
				logger->log_debug(name(), "drawed element in buffer");
			}
		}
	} catch (fawkes::Exception &e) {
		logger->log_error(name(), e);
	}
}

std::list<firevision::ROI> *
PuckVisionThread::classifyInRoi(firevision::ROI searchArea, color_classifier_context_t_ *color_data)
{
	try {
		color_data->scanline_grid->reset();
		color_data->scanline_grid->set_roi(&searchArea);
		color_data->classifier->set_src_buffer(buffer_,
		                                       camera_info_.img_width,
		                                       camera_info_.img_height);

		std::list<firevision::ROI> *ROIs = color_data->classifier->classify();

		return ROIs;
	} catch (fawkes::Exception &e) {
		logger->log_error(name(), e);
	}
	return new std::list<firevision::ROI>; // return a empty list if a exception
	                                       // is thrown
}

float
PuckVisionThread::getX(firevision::ROI *roi)
{
	// Distance X
	int   roiPositionY = camera_info_.img_height - (roi->start.y + roi->height / 2);
	float angle =
	  ((float)roiPositionY * camera_info_.opening_angle_vertical) / (float)camera_info_.img_height;
	float distance_x = camera_info_.position_z
	                   * (tan(angle + camera_info_.angle_horizontal_to_opening_)
	                      - tan(camera_info_.angle_horizontal_to_opening_));

	// logger->log_debug(name(),"angle: %f, image_size x: %i y: %i,roi x: %i y:
	// %i", (angle*180) / M_PI, img_width_, img_height_, roi->start.x,
	// roi->start.y);

	return distance_x + camera_info_.offset_cam_x_to_groundplane_;
	;
}

float
PuckVisionThread::getY(firevision::ROI *roi)
{
	// Left-Right
	int   roiPositionX = (roi->start.x + roi->width / 2);
	float alpha        = ((((float)camera_info_.img_width / 2) - (float)roiPositionX)
                 / ((float)camera_info_.img_width / 2))
	              * (camera_info_.opening_angle_horizontal / 2);
	float x               = getX(roi);
	float position_y_in_m = sin(alpha * x);

	// logger->log_debug(name(),"angle: %f, roi pos X: %i, distance y: %f, cfg
	// %f", alpha, roiPositionX, position_y_in_m,
	// cfg_camera_opening_angle_horizontal_);

	return position_y_in_m;
}

void
PuckVisionThread::getPuckPosition(puck *p, firevision::ROI roi)
{
	// Left-Right
	int   roiPositionX_L = (roi.start.x);
	int   roiPositionX_R = (roi.start.x + roi.width);
	float alpha_L        = ((((float)camera_info_.img_width / 2) - (float)roiPositionX_L)
                   / ((float)camera_info_.img_width / 2))
	                * (camera_info_.opening_angle_horizontal / 2);
	float alpha_R = ((((float)camera_info_.img_width / 2) - (float)roiPositionX_R)
	                 / ((float)camera_info_.img_width / 2))
	                * (camera_info_.opening_angle_horizontal / 2);
	float x                 = getX(&roi);
	float position_y_in_m_L = sin(alpha_L * x);
	float position_y_in_m_R = sin(alpha_R * x);

	p->roi = roi;

	p->cart[0]       = getX(&roi);
	p->cart[1]       = getY(&roi);
	p->cart[2]       = 0;
	p->cart.stamp    = fawkes::Time();
	p->cart.frame_id = camera_info_.frame;
	p->cart          = apply_tf(cfg_frame_target.c_str(), p->cart);

	cartToPol(p->pol, p->cart[0], p->cart[1]);
	p->visibility_history = 1;
	p->radius             = position_y_in_m_L - position_y_in_m_R;

	if (cfg_debugMessagesActivated_) {
		logger->log_info(name(), "puck: r: %f pol: phi %f, r%f", p->radius, p->pol.phi, p->pol.r);
	}
}

void
PuckVisionThread::updatePos3dInferface(fawkes::Position3DInterface *interface, puck *p)
{
	int visibility_history = interface->visibility_history();
	if (p) {
		interface->set_translation(0, p->cart[0]);
		interface->set_translation(1, p->cart[1]);
		interface->set_translation(2, p->cart[2]);
		interface->set_rotation(0, p->pol.phi);
		interface->set_rotation(1, p->pol.r);
		// interface->set_timestamp(&p->cart.stamp);
		interface->set_frame(p->cart.frame_id.c_str());
		interface->set_visibility_history(p->visibility_history);
		if (visibility_history >= 0) {
			interface->set_visibility_history(visibility_history + 1);
		} else {
			interface->set_visibility_history(1);
		}
	} else {
		// puck not visible
		if (visibility_history <= 0) {
			interface->set_visibility_history(visibility_history - 1);
		} else {
			interface->set_visibility_history(-1);
		}
	}
}

void
PuckVisionThread::updateInterface(std::vector<puck> *pucks)
{
	/* Create missing interfaces */
	if (pucks->size() > puck_interfaces_.size()) {
		unsigned int nr_missing_interfaces = pucks->size() - puck_interfaces_.size();
		for (unsigned int i = 0; i < nr_missing_interfaces; ++i) {
			createPuckInterface();
		}
	}

	if (cfg_debugMessagesActivated_) {
		logger->log_info(name(), "Detected pucks in view:  %i", pucks->size());
	}

	std::vector<fawkes::Position3DInterface *>::iterator interface_it = puck_interfaces_.begin();
	std::vector<puck>::iterator                          puck_it      = pucks->begin();
	for (; interface_it != puck_interfaces_.end(); ++interface_it) {
		fawkes::Position3DInterface *interface = (*interface_it);
		if (puck_it != pucks->end()) {
			updatePos3dInferface(interface, &(*puck_it));
			++puck_it;
		} else {
			updatePos3dInferface(interface, NULL);
		}
		interface->write();
	}
}

void
PuckVisionThread::finalize() // TODO check if everthing gets deleted
{
	logger->log_debug(name(), "finalize starts");

	vision_master->unregister_thread(this);
	delete shm_buffer_;
	config->rem_change_handler(this);
	delete cam_;
	cam_ = NULL;

	fawkes::Position3DInterface *interface;
	while (!puck_interfaces_.empty()) {
		interface = puck_interfaces_.back();
		blackboard->close(interface);
		puck_interfaces_.pop_back();
	}

	deleteClassifier(&puck_info_.main);
	deleteClassifier(&puck_info_.top_dots);

	logger->log_info(name(), "finalize ends");
}

void PuckVisionThread::config_value_erased(const char *path){};
void PuckVisionThread::config_tag_changed(const char *new_tag){};
void PuckVisionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v){};
void
PuckVisionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{
	if (cfg_mutex_.try_lock()) {
		try {
			loadConfig();
			init_with_config();
		} catch (fawkes::Exception &e) {
			logger->log_error(name(), e);
		}
	}
	// gets called for every changed entry... so init is called once per change.
	cfg_mutex_.unlock();
}

Point3d
PuckVisionThread::apply_tf(const char *target_frame, Point3d src)
{
	Point3d targetPoint;
	targetPoint.frame_id     = target_frame;
	const char *source_frame = src.frame_id.c_str();

	bool link_frame_exists  = tf_listener->frame_exists(target_frame);
	bool laser_frame_exists = tf_listener->frame_exists(source_frame);

	if (!link_frame_exists || !laser_frame_exists) {
		logger->log_warn(name(),
		                 "Frame missing: %s %s   %s %s",
		                 source_frame,
		                 link_frame_exists ? "exists" : "missing",
		                 target_frame,
		                 laser_frame_exists ? "exists" : "missing");
	} else {
		try {
			tf_listener->transform_point(target_frame, src, targetPoint);
		} catch (fawkes::tf::ExtrapolationException &e) {
			logger->log_debug(name(), "Extrapolation error: %s", e.what());
			return src;
		} catch (fawkes::tf::ConnectivityException &e) {
			logger->log_debug(name(), "Connectivity exception: %s", e.what());
			return src;
		} catch (fawkes::Exception &e) {
			logger->log_debug(name(), "Fawkes exception: %s", e.what());
			return src;
		} catch (std::exception &e) {
			logger->log_debug(name(), "Generic exception: %s", e.what());
			return src;
		}
		return targetPoint;
	}
	return src;
}
