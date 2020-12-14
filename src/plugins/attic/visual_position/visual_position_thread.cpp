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

#include "visual_position_thread.h"

VisualPositionThread::VisualPositionThread()
: Thread("VisualPositionThread", Thread::OPMODE_WAITFORWAKEUP),
  VisionAspect(VisionAspect::CYCLIC),
  ConfigurationChangeHandler(CFG_PREFIX)
{
	cfg_prefix_           = CFG_PREFIX;
	cfg_frame_camera_pos_ = "";

	camera_info_.cfg_camera_               = "";
	camera_info_.img_width_                = 0;
	camera_info_.img_height_               = 0;
	camera_info_.opening_angle_horizontal_ = 0;
	camera_info_.opening_angle_vertical_   = 0;

	buffer_color_     = NULL;
	buffer_gray_      = NULL;
	buffer_gray_edgy_ = NULL;

	shm_buffer_color_     = NULL;
	shm_buffer_gray_      = NULL;
	shm_buffer_gray_edgy_ = NULL;

	cspaceFrom_ = firevision::YUV422_PLANAR;
	cspaceTo_   = firevision::YUV422_PLANAR;

	cam_ = NULL;

	switchInterface_ = NULL;
	no_pucK_         = NULL;

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
VisualPositionThread::createPuckInterface()
{
	fawkes::Position3DInterface *puck_if_ = NULL;

	std::string interface_name = "puck_" + std::to_string(puck_interfaces_.size());
	logger->log_info(name(), "Creating new Position3DInterface %s", interface_name.c_str());

	try {
		puck_if_ = blackboard->open_for_writing<fawkes::Position3DInterface>(interface_name.c_str());
		puck_if_->set_visibility_history(-9999);
		puck_if_->set_frame("/base_link");
		puck_if_->write();
		puck_interfaces_.push_back(puck_if_);
	} catch (std::exception &e) {
		finalize();
		throw;
	}
}

void
VisualPositionThread::loadConfig()
{
	logger->log_info(name(), "loading config");
	cfg_prefix_                   = CFG_PREFIX;
	cfg_prefix_static_transforms_ = config->get_string((cfg_prefix_ + "transform").c_str());

	cfg_frame_camera_pos_ =
	  config->get_string((cfg_prefix_static_transforms_ + "child_frame").c_str());

	// load camera informations
	camera_info_.cfg_camera_ = config->get_string((cfg_prefix_ + "camera").c_str());
	camera_info_.position_x_ = config->get_float((cfg_prefix_static_transforms_ + "trans_x").c_str());
	camera_info_.position_y_ = config->get_float((cfg_prefix_static_transforms_ + "trans_y").c_str());
	camera_info_.position_z_ = config->get_float((cfg_prefix_static_transforms_ + "trans_z").c_str());
	camera_info_.position_pitch_ =
	  config->get_float((cfg_prefix_static_transforms_ + "rot_pitch").c_str());
	camera_info_.opening_angle_horizontal_ =
	  config->get_float((cfg_prefix_ + "camera_opening_angle_horizontal").c_str());
	camera_info_.opening_angle_vertical_ =
	  config->get_float((cfg_prefix_ + "camera_opening_angle_vertical").c_str());
	// Calculate visible Area
	camera_info_.angle_horizontal_to_opening_ =
	  (1.57 - camera_info_.position_pitch_) - (camera_info_.opening_angle_vertical_ / 2);
	camera_info_.visible_lenght_x_in_m_ =
	  camera_info_.position_z_
	  * (tan(camera_info_.opening_angle_vertical_ + camera_info_.angle_horizontal_to_opening_)
	     - tan(camera_info_.angle_horizontal_to_opening_));
	camera_info_.offset_cam_x_to_groundplane_ =
	  camera_info_.position_z_ * tan(camera_info_.angle_horizontal_to_opening_);
	camera_info_.visible_lenght_y_in_m_ =
	  camera_info_.position_z_ * tan(camera_info_.opening_angle_horizontal_ / 2);

	logger->log_debug(name(),
	                  "Camera opening angles Horizontal: %f Vertical: %f",
	                  camera_info_.opening_angle_horizontal_,
	                  camera_info_.opening_angle_vertical_);

	cfg_debugMessagesActivated_ = config->get_bool((cfg_prefix_ + "show_debug_messages").c_str());
	cfg_paintROIsActivated_     = config->get_bool((cfg_prefix_ + "draw_rois").c_str());

	search_area.start.x = config->get_uint((cfg_prefix_ + "search_area/start/x").c_str());
	search_area.start.y = config->get_uint((cfg_prefix_ + "search_area/start/y").c_str());
	search_area.width   = config->get_uint((cfg_prefix_ + "search_area/width").c_str());
	search_area.height  = config->get_uint((cfg_prefix_ + "search_area/height").c_str());
	search_area.color   = firevision::C_BLUE;

	// Config Value for the classifier mode
	cfg_colormodel_mode_ = config->get_string((cfg_prefix_ + "colormodel_mode").c_str());
}

void
VisualPositionThread::init_with_config()
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
		cam_ = vision_master->register_for_camera(camera_info_.cfg_camera_.c_str(), this);
		cam_->start();
		cam_->open();
		camera_info_.img_width_  = cam_->pixel_width();
		camera_info_.img_height_ = cam_->pixel_height();

		search_area.image_height = camera_info_.img_height_;
		search_area.image_width  = camera_info_.img_width_;
		printRoi(search_area);

		cspaceFrom_ = cam_->colorspace();

		// SHM image buffer
		if (shm_buffer_color_ != NULL) {
			delete shm_buffer_color_;
			shm_buffer_color_ = NULL;
			buffer_color_     = NULL;

			delete shm_buffer_gray_;
			shm_buffer_gray_ = NULL;
			buffer_gray_     = NULL;

			delete shm_buffer_gray_edgy_;
			shm_buffer_gray_edgy_ = NULL;
			buffer_gray_edgy_     = NULL;
		}
		std::string shmID = config->get_string((cfg_prefix_ + "shm_image_id").c_str());
		shm_buffer_color_ = new firevision::SharedMemoryImageBuffer(shmID.c_str(),
		                                                            cspaceTo_,
		                                                            camera_info_.img_width_,
		                                                            camera_info_.img_height_);

		shmID            = shmID + "_gray";
		shm_buffer_gray_ = new firevision::SharedMemoryImageBuffer(shmID.c_str(),
		                                                           firevision::GRAY8,
		                                                           camera_info_.img_width_,
		                                                           camera_info_.img_height_);

		shmID                 = shmID + "_edgy";
		shm_buffer_gray_edgy_ = new firevision::SharedMemoryImageBuffer(shmID.c_str(),
		                                                                firevision::GRAY8,
		                                                                camera_info_.img_width_,
		                                                                camera_info_.img_height_);

		if (!shm_buffer_color_->is_valid() || !shm_buffer_gray_->is_valid()
		    || !shm_buffer_gray_edgy_->is_valid()) {
			delete shm_buffer_color_;
			delete shm_buffer_gray_;
			delete cam_;
			shm_buffer_color_ = NULL;
			shm_buffer_gray_  = NULL;
			cam_              = NULL;
			throw fawkes::Exception("Shared memory segment not valid");
		}

		shm_buffer_color_->set_frame_id(cfg_frame_camera_pos_.c_str());
		shm_buffer_gray_->set_frame_id(cfg_frame_camera_pos_.c_str());
		shm_buffer_gray_edgy_->set_frame_id(cfg_frame_camera_pos_.c_str());

		buffer_color_     = shm_buffer_color_->buffer();
		buffer_gray_      = shm_buffer_gray_->buffer();
		buffer_gray_edgy_ = shm_buffer_gray_edgy_->buffer();

		camera_info_.fullimage =
		  firevision::ROI::full_image(camera_info_.img_width_, camera_info_.img_height_);
	}

	//	//load puck infos;
	puck_info_.radius = config->get_float((cfg_prefix_ + "puck/radius").c_str());
	puck_info_.height = config->get_float((cfg_prefix_ + "puck/height").c_str());

	// Configure classifier
	logger->log_info(name(), "Loading colormodel %s", cfg_colormodel_mode_.c_str());

	setup_color_classifier(&puck_info_.main, "puck/red", firevision::C_RED);
	setup_color_classifier(&puck_info_.top_dots, "puck/topdots", firevision::C_YELLOW);

	logger->log_debug(name(),
	                  "Visible X: %f y: %f Offset cam groundplane: %f angle "
	                  "(horizontal/opening): %f ",
	                  camera_info_.visible_lenght_x_in_m_,
	                  camera_info_.visible_lenght_y_in_m_,
	                  camera_info_.offset_cam_x_to_groundplane_,
	                  camera_info_.angle_horizontal_to_opening_);
	logger->log_debug(name(),
	                  "cam transform X: %f y: %f z: %f pitch: %f",
	                  camera_info_.position_x_,
	                  camera_info_.position_y_,
	                  camera_info_.position_z_,
	                  camera_info_.position_pitch_);
}

void
VisualPositionThread::init()
{
	logger->log_info(name(), "starts init");

	config->add_change_handler(this);

	no_pucK_                    = new puck();
	int val_empty               = -9999;
	no_pucK_->visibiity_history = val_empty;
	no_pucK_->cart.x            = val_empty;
	no_pucK_->cart.y            = val_empty;
	no_pucK_->cart.z            = val_empty;
	drawer_                     = new firevision::FilterROIDraw();
	loadConfig();
	init_with_config();
	createPuckInterface();

	logger->log_debug(name(), "end of init()");
}

void
VisualPositionThread::deleteClassifier(color_classifier_context_t_ *color_data)
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
VisualPositionThread::setup_color_classifier(color_classifier_context_t_ *color_data,
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

	color_data->scanline_grid = new firevision::ScanlineGrid(camera_info_.img_width_,
	                                                         camera_info_.img_height_,
	                                                         color_data->cfg_scangrid_x_offset,
	                                                         color_data->cfg_scangrid_y_offset);

	if (cfg_colormodel_mode_ == "colormap") {
		std::string lutname = std::string(prefix) + "/lut"; //"puck_vison/" +
		std::string filename =
		  std::string(CONFDIR) + "/" + config->get_string((cfg_prefix_ + prefix + "/lut").c_str());
		color_data->colormodel = new firevision::ColorModelLookupTable(filename.c_str(),
		                                                               lutname.c_str(),
		                                                               true /* destroy on delete */);

		color_data->classifier =
		  new firevision::SimpleColorClassifier(color_data->scanline_grid,
		                                        color_data->colormodel,
		                                        color_data->cfg_roi_min_points,
		                                        color_data->cfg_roi_basic_size,
		                                        false,
		                                        color_data->cfg_roi_neighborhood_min_match,
		                                        0);

	} else if (cfg_colormodel_mode_ == "similarity") {
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
	} else {
		throw new fawkes::Exception("selected colormodel not supported");
	}
}

void
VisualPositionThread::drawRois(std::list<firevision::ROI> *rois_)
{
	if (cfg_paintROIsActivated_) {
		for (std::list<firevision::ROI>::iterator list_iter = rois_->begin(); list_iter != rois_->end();
		     list_iter++) {
			drawROIIntoBuffer(*list_iter, firevision::FilterROIDraw::DASHED_HINT);
		}
	}
}

void
VisualPositionThread::mergeWithColorInformation(firevision::color_t         color,
                                                std::list<firevision::ROI> *rois_color_,
                                                std::list<firevision::ROI> *rois_all)
{
	while (rois_color_->size() != 0) {
		rois_color_->front().color = color;
		rois_all->push_front(rois_color_->front());
		rois_color_->pop_front();
	}
}

int
schnittpunkt(myLine line1, myLine line2, firevision::ROI &result)
{
	using namespace std;
	// Die 4 Punkte, p1 und p2 ergeben Linie 1 und p3, p4 ergeben Linie 2
	int p1[] = {(int)line1.upper_point.start.x, (int)line1.upper_point.start.y, 1};
	int p2[] = {(int)line1.lower_point.start.x, (int)line1.lower_point.start.y, 1};
	int p3[] = {(int)line2.upper_point.start.x, (int)line2.upper_point.start.y, 1};
	int p4[] = {(int)line2.lower_point.start.x, (int)line2.lower_point.start.y, 1};

	int    l1[3], l2[3], s[3];
	double sch[2];
	l1[0]  = p1[1] * p2[2] - p1[2] * p2[1];
	l1[1]  = p1[2] * p2[0] - p1[0] * p2[2];
	l1[2]  = p1[0] * p2[1] - p1[1] * p2[0];
	l2[0]  = p3[1] * p4[2] - p3[2] * p4[1];
	l2[1]  = p3[2] * p4[0] - p3[0] * p4[2];
	l2[2]  = p3[0] * p4[1] - p3[1] * p4[0];
	s[0]   = l1[1] * l2[2] - l1[2] * l2[1];
	s[1]   = l1[2] * l2[0] - l1[0] * l2[2];
	s[2]   = l1[0] * l2[1] - l1[1] * l2[0];
	sch[0] = (double)s[0] / (double)s[2];
	sch[1] = (double)s[1] / (double)s[2];

	result.start.x = (unsigned int)sch[0];
	result.start.y = (unsigned int)sch[1];

	cout << "Ergebnis: x" << sch[0] << " y: " << sch[1] << endl;

	return 0;
}

void
VisualPositionThread::loop()
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
		                    buffer_color_,
		                    camera_info_.img_width_,
		                    camera_info_.img_height_);

		cam_->dispose_buffer();
		IplImage *p_image =
		  cvCreateImage(cvSize(camera_info_.img_width_, camera_info_.img_height_), IPL_DEPTH_8U, 3);
		firevision::IplImageAdapter::convert_image_bgr(cam_->buffer(), p_image);
		//
		cv::Mat image_edgy;
		cv::Mat image_colored(p_image, false);
		cv::cvtColor(image_colored, image_edgy, CV_BGR2GRAY);
		// cv::imwrite("edgy_grey.jpg",image_edgy);
		// cv::threshold(image_edgy, image_edgy, 70, 255, cv::THRESH_BINARY );
		// cv::imwrite("edgy_threshold.jpg",image_edgy);
		//
		// cv::imshow("cam",m);

		cv::medianBlur(image_edgy, image_edgy, 3);
		cv::imwrite("edgy_clean_median.jpg", image_edgy);
		cv::Canny(image_edgy, image_edgy, 200, 230, 3, true);
		cv::imwrite("edgy_clean.jpg", image_edgy);
		// cv::imshow("edge",image_edgy);
		cv::imwrite("edgy_threshold.jpg", image_edgy);

		//		std::vector<cv::Vec2f> lines;
		//		cv::HoughLines(m, lines, 1, CV_PI/180, 100, 0, 0 );
		//		std::vector<cv::Vec2f>::iterator lineshape_it = lines.begin();

		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(image_edgy, lines, 1, CV_PI / (180 * 4), 50, 60, 30);
		std::vector<cv::Vec4i>::iterator lineshape_it = lines.begin();
		//

		// delete p_image;

		// cv::imshow("lines",image_edgy);
		// cv::waitKey(2);

		std::vector<myLine> myLines;

		logger->log_info(name(), "%i Lines", lines.size());
		for (; lineshape_it != lines.end(); lineshape_it++) {
			myLine line(search_area, search_area);
			line.set(*lineshape_it, 5);

			double angle_abs = sqrt(line.angle * line.angle); // abs(angle);
			logger->log_info(name(), "Angle: %f %f", line.angle, angle_abs);

			if (angle_abs > 70 && angle_abs < 110) {
				// calculate positions of roi
				// Bottem line should be 16 cm
				// line height 6 cm

				if (search_area.contains(line.lower_point.start.x, line.lower_point.start.y)
				    && search_area.contains(line.upper_point.start.x, line.upper_point.start.y)) {
					myLines.push_back(line);
					printRoi(line.lower_point);
					printRoi(line.upper_point);

					drawROIIntoBuffer(line.lower_point);
					drawROIIntoBuffer(line.upper_point);
					line.drawCV(image_colored, cv::Scalar(0, 0, 255));
				} else {
					line.drawCV(image_colored, cv::Scalar(255, 0, 255));
				}
			} else {
				line.drawCV(image_colored, cv::Scalar(0, 255, 255));
			}

			// logger->log_info(name(), "rois in buffer");
		}

		logger->log_info(name(), "%i Lines", myLines.size());

		if (myLines.size() == 2) {
			myLine bottem(myLines[0].lower_point, myLines[1].lower_point);
			bottem.drawCV(image_colored, cv::Scalar(0, 255, 0));

			myLine top(myLines[0].upper_point, myLines[1].upper_point);
			top.drawCV(image_colored, cv::Scalar(0, 255, 0));

			myLine diag1(myLines[0].upper_point, myLines[1].lower_point);
			diag1.drawCV(image_colored, cv::Scalar(0, 255, 0));

			myLine diag2(myLines[0].lower_point, myLines[1].upper_point);
			diag2.drawCV(image_colored, cv::Scalar(0, 255, 0));

			firevision::ROI center = getCenterRoi(bottem.lower_point, bottem.upper_point);
			cv::circle(image_colored,
			           cv::Point(center.start.x - 1, center.start.y - 1),
			           3,
			           cv::Scalar(255, 0, 0),
			           1);

			schnittpunkt(diag1, diag2, center);

			center.color = firevision::C_BLUE;
			drawROIIntoBuffer(center);
		}

		cv::imwrite("edgy_lines.jpg", image_colored);
		drawROIIntoBuffer(search_area);

		//		firevision::grayscale(cspaceFrom_,
		//					cam_->buffer(),
		//					buffer_gray_,
		//					camera_info_.img_width_,
		//					camera_info_.img_height_);

		//		firevision::FilterSobel edge_filter;
		//		edge_filter.set_src_buffer(buffer_gray_, &search_area,
		// firevision::ORI_HORIZONTAL);
		// edge_filter.set_dst_buffer(buffer_gray_edgy_, &search_area);
		// edge_filter.apply();

		//		firevision::convert(firevision::GRAY8,
		//							cspaceTo_,
		//							buffer_gray_edgy_,
		//							buffer_color_,
		//							camera_info_.img_width_,
		//							camera_info_.img_height_);

		//		firevision::RhtLinesModel model;
		//		int nrFirevision = model.parseImage(buffer_gray_, &search_area);
		//		logger->log_info(name(),"found lines: %i %i", nrFirevision,
		// model.getShapeCount());

		//		std::vector<firevision::LineShape>* lineshapes =
		// model.getShapes();

		//		std::vector<firevision::ROI> rois;
		//		for(std::vector<firevision::LineShape>::iterator lineshape_it =
		// lineshapes->begin(); 				lineshape_it !=
		// lineshapes->end(); lineshape_it++)
		//		{
		//			int w = 10;
		//			int h = 10;
		//
		//			firevision::ROI r1(search_area);
		//			r1.height = h;
		//			r1.width = w;
		//
		//			firevision::ROI r2(search_area);
		//			r2.height = h;
		//			r2.width = w;
		//			(*lineshape_it).calcPoints();
		//			int points[4];
		//			(*lineshape_it).getPoints(&points[0], &points[1],
		//&points[2], &points[3]); 			r1.start.x = (unsigned int)
		// points[0]; 			r1.start.y = (unsigned int) points[1];
		// r2.start.x = (unsigned int) points[2]; 			r2.start.y =
		//(unsigned int) points[3];
		//
		//			r1.color = firevision::C_BLUE;
		//			r2.color = firevision::C_RED;
		//
		//			printRoi(r1);
		//			printRoi(r2);
		//
		//			drawROIIntoBuffer(r1);
		//			drawROIIntoBuffer(r2);
		//
		//			logger->log_info(name(), "rois in buffer");
		//		}
	}
	cfg_mutex_.unlock();
}

void
VisualPositionThread::sortPucks(std::vector<puck> *pucks)
{
	// Sort pucks
	std::sort(pucks->begin(), pucks->end(), [](const puck a, const puck &b) -> bool {
		return a.pol.r < b.pol.r;
	});
}

void
VisualPositionThread::calculatePuckPositions(std::vector<puck> *        pucks,
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
VisualPositionThread::printRoi(firevision::ROI roi)
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
VisualPositionThread::fitROI(firevision::ROI &roi, int x, int y, int w, int h)
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

firevision::ROI *
VisualPositionThread::getRoiContainingRoi(std::list<firevision::ROI> *roiList,
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
VisualPositionThread::getBiggestRoi(std::list<firevision::ROI> *roiList)
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

VisualPositionThread::~VisualPositionThread()
{
	// TODO Auto-generated destructor stub
}

void
VisualPositionThread::cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y)
{
	pol.phi = atan2f(y, x);
	pol.r   = sqrtf(x * x + y * y);

	if (cfg_debugMessagesActivated_) {
		logger->log_debug(name(), "x: %f; y: %f", x, y);
		logger->log_debug(name(), "Calculated r: %f; phi: %f", pol.r, pol.phi);
	}
}

void
VisualPositionThread::polToCart(float &x, float &y, fawkes::polar_coord_2d_t pol)
{
	x = pol.r * std::cos(pol.phi);
	y = pol.r * std::sin(pol.phi);
}

void
VisualPositionThread::drawROIIntoBuffer(firevision::ROI                           roi,
                                        firevision::FilterROIDraw::border_style_t borderStyle)
{
	try {
		if (cfg_paintROIsActivated_) {
			drawer_->set_src_buffer(buffer_color_,
			                        firevision::ROI::full_image(camera_info_.img_width_,
			                                                    camera_info_.img_height_),
			                        0);
			drawer_->set_dst_buffer(buffer_color_, &roi);
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
VisualPositionThread::classifyInRoi(firevision::ROI              searchArea,
                                    color_classifier_context_t_ *color_data)
{
	try {
		color_data->scanline_grid->reset();
		color_data->scanline_grid->set_roi(&searchArea);
		color_data->classifier->set_src_buffer(buffer_color_,
		                                       camera_info_.img_width_,
		                                       camera_info_.img_height_);

		std::list<firevision::ROI> *ROIs = color_data->classifier->classify();

		return ROIs;
	} catch (fawkes::Exception &e) {
		logger->log_error(name(), e);
	}
	return new std::list<firevision::ROI>; // return a empty list if a exception
	                                       // is thrown
}

float
VisualPositionThread::getX(firevision::ROI *roi)
{
	// Distance X
	int   roiPositionY = camera_info_.img_height_ - (roi->start.y + roi->height / 2);
	float angle =
	  ((float)roiPositionY * camera_info_.opening_angle_vertical_) / (float)camera_info_.img_height_;
	float distance_x = camera_info_.position_z_
	                   * (tan(angle + camera_info_.angle_horizontal_to_opening_)
	                      - tan(camera_info_.angle_horizontal_to_opening_));

	// logger->log_debug(name(),"angle: %f, image_size x: %i y: %i,roi x: %i y:
	// %i", (angle*180) / M_PI, img_width_, img_height_, roi->start.x,
	// roi->start.y);

	return distance_x + camera_info_.offset_cam_x_to_groundplane_ + camera_info_.position_x_;
}

float
VisualPositionThread::getY(firevision::ROI *roi)
{
	// Left-Right
	int   roiPositionX = (roi->start.x + roi->width / 2);
	float alpha        = ((((float)camera_info_.img_width_ / 2) - (float)roiPositionX)
                 / ((float)camera_info_.img_width_ / 2))
	              * (camera_info_.opening_angle_horizontal_ / 2);
	float x               = getX(roi);
	float position_y_in_m = sin(alpha * x);

	// logger->log_debug(name(),"angle: %f, roi pos X: %i, distance y: %f, cfg
	// %f", alpha, roiPositionX, position_y_in_m,
	// cfg_camera_opening_angle_horizontal_);

	return position_y_in_m;
}

void
VisualPositionThread::getPuckPosition(puck *p, firevision::ROI roi)
{
	// Left-Right
	int   roiPositionX_L = (roi.start.x);
	int   roiPositionX_R = (roi.start.x + roi.width);
	float alpha_L        = ((((float)camera_info_.img_width_ / 2) - (float)roiPositionX_L)
                   / ((float)camera_info_.img_width_ / 2))
	                * (camera_info_.opening_angle_horizontal_ / 2);
	float alpha_R = ((((float)camera_info_.img_width_ / 2) - (float)roiPositionX_R)
	                 / ((float)camera_info_.img_width_ / 2))
	                * (camera_info_.opening_angle_horizontal_ / 2);
	float x                 = getX(&roi);
	float position_y_in_m_L = sin(alpha_L * x);
	float position_y_in_m_R = sin(alpha_R * x);

	p->roi    = roi;
	p->cart.x = getX(&roi);
	p->cart.y = getY(&roi);
	p->cart.z = 0;
	cartToPol(p->pol, p->cart.x, p->cart.y);
	p->visibiity_history = 1;
	p->radius            = position_y_in_m_L - position_y_in_m_R;

	if (cfg_debugMessagesActivated_) {
		logger->log_info(name(), "puck: r: %f pol: phi %f, r%f", p->radius, p->pol.phi, p->pol.r);
	}
}

void
VisualPositionThread::updatePos3dInferface(fawkes::Position3DInterface *interface, puck *p)
{
	interface->set_translation(0, p->cart.x);
	interface->set_translation(1, p->cart.y);
	interface->set_translation(2, p->cart.z);
	interface->set_rotation(0, p->pol.phi);
	interface->set_rotation(1, p->pol.r);
	interface->set_visibility_history(p->visibiity_history);
}

void
VisualPositionThread::updateInterface(std::vector<puck> *pucks)
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
			interface->write();
			++puck_it;
		} else {
			updatePos3dInferface(interface, no_pucK_);
			interface->write();
		}
	}
}

void
VisualPositionThread::finalize() // TODO check if everthing gets deleted
{
	logger->log_debug(name(), "finalize starts");

	vision_master->unregister_thread(this);
	delete shm_buffer_color_;
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

double
center(double e1, double e2)
{
	double delta = e2 - e1;
	return e1 + delta / 2;
}

firevision::ROI
VisualPositionThread::getCenterRoi(firevision::ROI p1, firevision::ROI p2)
{
	const char *frame_cam  = "/cam_front_puck";
	const char *frame_base = "/base_link";
	Point3d     stamped_p1;
	stamped_p1[0]       = p1.start.x;
	stamped_p1[1]       = p1.start.y;
	stamped_p1[2]       = 0;
	stamped_p1.frame_id = frame_cam;
	stamped_p1          = apply_tf(frame_base, stamped_p1);

	Point3d stamped_p2;
	stamped_p2[0]       = p2.start.x;
	stamped_p2[1]       = p2.start.y;
	stamped_p2[2]       = 0;
	stamped_p2.frame_id = frame_cam;

	stamped_p2 = apply_tf(frame_base, stamped_p2);

	Point3d myCenter;
	myCenter[0]       = center(stamped_p1[0], stamped_p2[0]);
	myCenter[1]       = center(stamped_p1[1], stamped_p2[1]);
	myCenter.frame_id = stamped_p2.frame_id;

	myCenter = apply_tf(frame_cam, myCenter);

	logger->log_info(name(), "x1: %f x2: %f cx:%f", stamped_p1[0], stamped_p2[0], myCenter[0]);

	firevision::ROI center(p1);
	center.height  = 2;
	center.width   = 2;
	center.start.x = myCenter[0];
	center.start.y = myCenter[1];

	return center;
};

void VisualPositionThread::config_value_erased(const char *path){};
void VisualPositionThread::config_tag_changed(const char *new_tag){};
void VisualPositionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v){};
void
VisualPositionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
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
VisualPositionThread::apply_tf(const char *target_frame, Point3d src)
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
